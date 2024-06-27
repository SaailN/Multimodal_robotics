import os
import sys

if "big_vision_repo" not in sys.path:
  sys.path.append("big_vision_repo")

import base64
import functools
import html
import io
import os
import warnings

import jax
import jax.numpy as jnp
import numpy as np
import ml_collections

import tensorflow as tf
import sentencepiece

from IPython.core.display import display, HTML
from PIL import Image
from tqdm.notebook import tqdm
from fastapi import FastAPI, File, UploadFile
from pydantic import BaseModel
import uvicorn

import re
import cv2
import json
import supervision as sv
from typing import List
import ml_collections
import matplotlib.pyplot as plt

# Import model definition from big_vision
from big_vision.models.proj.paligemma import paligemma
from big_vision.trainers.proj.paligemma import predict_fns
import big_vision.utils as bv_utils

# Import big vision utilities
import big_vision.datasets.jsonl
import big_vision.utils
import big_vision.sharding

# Don't let TF use the GPU or TPUs
tf.config.set_visible_devices([], "GPU")
tf.config.set_visible_devices([], "TPU")

backend = jax.lib.xla_bridge.get_backend()
print(f"JAX version:  {jax.__version__}")
print(f"JAX platform: {backend.platform}")
print(f"JAX devices:  {jax.device_count()}")

os.environ["DISABLE_NEST_ASYNCIO"]="True"

app = FastAPI()

MODEL_PATH = "eysip/model/paligemma-3b-pt-224.f16.npz"
TOKENIZER_PATH = "eysip/model/paligemma_tokenizer.model"
SEQLEN = 128
CLASSES = ['Apple', 'Banana', 'Carrot', 'Orange', 'Basket', 'Grape', 'Pineapple', 'Papaya', 'Mango', 'Pear', 'Watermelon', 'Cherry', 'Peach', 'Lime']

class Objects(BaseModel):
    objects : list

'''
Functions
'''

def preprocess_image(image, size=224):
  # Model has been trained to handle images of different aspects ratios
  # resized to 224x224 in the range [-1, 1]. Bilinear and antialias resize
  # options are helpful to improve quality in some tasks.
  image = np.asarray(image)
  if image.ndim == 2:  # Convert image without last channel into greyscale.
    image = np.stack((image,)*3, axis=-1)
  image = image[..., :3]  # Remove alpha layer.
  assert image.shape[-1] == 3

  image = tf.constant(image)
  image = tf.image.resize(image, (size, size), method='bilinear', antialias=True)
  return image.numpy() / 127.5 - 1.0  # [0, 255]->[-1,1]

def preprocess_tokens(prefix, suffix=None, seqlen=None):
  # Model has been trained to handle tokenized text composed of a prefix with
  # full attention and a suffix with causal attention.
  separator = "\n"
  tokens = tokenizer.encode(prefix, add_bos=True) + tokenizer.encode(separator)
  mask_ar = [0] * len(tokens)    # 0 to use full attention for prefix.
  mask_loss = [0] * len(tokens)  # 0 to not use prefix tokens in the loss.

  if suffix:
    suffix = tokenizer.encode(suffix, add_eos=True)
    tokens += suffix
    mask_ar += [1] * len(suffix)    # 1 to use causal attention for suffix.
    mask_loss += [1] * len(suffix)  # 1 to use suffix tokens in the loss.

  mask_input = [1] * len(tokens)    # 1 if it's a token, 0 if padding.
  if seqlen:
    padding = [0] * max(0, seqlen - len(tokens))
    tokens = tokens[:seqlen] + padding
    mask_ar = mask_ar[:seqlen] + padding
    mask_loss = mask_loss[:seqlen] + padding
    mask_input = mask_input[:seqlen] + padding

  return jax.tree.map(np.array, (tokens, mask_ar, mask_loss, mask_input))

def postprocess_tokens(tokens):
  tokens = tokens.tolist()  # np.array to list[int]
  try:  # Remove tokens at and after EOS if any.
    eos_pos = tokens.index(tokenizer.eos_id())
    tokens = tokens[:eos_pos]
  except ValueError:
    pass
  return tokenizer.decode(tokens)

def process_caption(caption):
  cap_split = caption.split(';')
  new_caption = ''
  i = 0
  for c in cap_split:
      c = c.strip()
      if (c == ''):
          continue
      cap_class = c.split(' ')[1]
      if (i == 0):
          new_str = c.split(' ')[0] + ' ' + cap_class.capitalize()
      else:
          new_str = '; ' + c.split(' ')[0] + ' ' + cap_class.capitalize()
      new_caption += new_str
      i+=1
  return new_caption

def get_classes(caption):
    cap_split = caption.split(';')
    new_caption = ''
    i = 0
    cap_class = []
    for c in cap_split:
        c = c.strip()
        if (c == ''):
            continue
        cap_class.append(c.split(' ')[1])
    return cap_class

def validation_data_iterator():
  """Single iterator over validation examples."""
  for example in val_dataset.get_tfdata(ordered=True).as_numpy_iterator():
    image = Image.open(io.BytesIO(example["image"]))
    image = preprocess_image(image)

    prefix = example["prefix"].decode().lower()
    suffix = example["suffix"].decode().lower()
    tokens, mask_ar, _, mask_input = preprocess_tokens(prefix, seqlen=SEQLEN)
    label, _, _, _ = preprocess_tokens(suffix, seqlen=SEQLEN)

    yield {
        "image": np.asarray(image),
        "text": np.asarray(tokens),
        "label": np.asarray(label),
        "mask_ar": np.asarray(mask_ar),
        "mask_input": np.asarray(mask_input),
    }

def make_single_image_prediction(image, seqlen=SEQLEN, sampler="greedy"):
    # Preprocess the image
    preprocessed_image = preprocess_image(image)
    example = {"image": preprocessed_image, "prefix" : "detect Apple ; Orange ; Banana ; Carrot ; Basket ; Pineapple ; Mango ; Peach ; Pear ; Cherry ; Lime ; Watermelon ; Papaya"}

    # Create a single-item iterator
    def single_item_iterator():
        prefix = example["prefix"]
        tokens, mask_ar, _, mask_input = preprocess_tokens(prefix, seqlen=SEQLEN)
        yield {
        "image": np.asarray(image),
        "text": np.asarray(tokens),
        "mask_ar": np.asarray(mask_ar),
        "mask_input": np.asarray(mask_input),
    }

    data_iterator = iter(single_item_iterator())

    # Use the provided function adapted for a single example
    outputs = []
    examples = []

    try:
        examples.append(next(data_iterator))
        examples[-1]["_mask"] = np.array(True)  # Indicates true example.
    except StopIteration:
        return outputs

    # Convert list of examples into a dict of np.arrays and load onto devices.
    batch = jax.tree.map(lambda *x: np.stack(x), *examples)
    batch = big_vision.utils.reshard(batch, data_sharding)

    # Make model predictions
    tokens = decode({"params": model_wts}, batch=batch,
                    max_decode_len=224, sampler=sampler)

    # Fetch model predictions to device and detokenize.
    tokens, mask = jax.device_get((tokens, batch["_mask"]))
    tokens = tokens[mask]  # remove padding examples.
    responses = [postprocess_tokens(t) for t in tokens]

    # Append to output
    for example, response in zip(examples, responses):
        outputs.append((example["image"], response))
    return outputs

def get_coordinates(caption):
    cap_split = caption.split(';')
    coor = []
    for c in cap_split:
        c = c.strip()
        if (c == ''):
          continue
        cap_coor = c.split(' ')[0]
        numbers = re.findall(r'\d+', cap_coor)
        coor.append(list(map(int, numbers)))
    return coor

model_wts = bv_utils.load_checkpoint_np(MODEL_PATH)
model_config = ml_collections.FrozenConfigDict({
    "llm": {"vocab_size": 257_152},
    "img": {"variant": "So400m/14", "pool_type": "none", "scan": True, "dtype_mm": "float16"}
})
model = paligemma.Model(**model_config)
tokenizer = sentencepiece.SentencePieceProcessor(TOKENIZER_PATH)

mesh = jax.sharding.Mesh(jax.devices(), ("data"))
data_sharding = jax.sharding.NamedSharding(
    mesh, jax.sharding.PartitionSpec("data"))

decode_fn = predict_fns.get_all(model)['decode']
decode = functools.partial(decode_fn, devices=jax.devices(), eos_token=tokenizer.eos_id())

@app.post("/image/")
async def predict(file : UploadFile = File(...)):
    image = await file.read()
    nparr = np.frombuffer(image, np.uint8)
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    h, w, _ = image.shape

    location = ""
    image = preprocess_image(image)
    for image, caption in make_single_image_prediction(image):
        caption = process_caption(caption)
        location = caption

    c = get_coordinates(location)
    classes = get_classes(location)
    for i in range(len(c)):
        for j in range(len(c[0])):
            c[i][j] = c[i][j] / 1024
            if (j % 2 == 0):
                c[i][j] *= h
            else:
                c[i][j] *= w
            c[i][j] = int(c[i][j])

    print(c)

    output = []
    for i, cl in enumerate(classes):
        output.append({'object':cl, 'location':c[i]})
    print(output)

    return {"image": output}

if __name__ == '__main__':
   uvicorn.run(app)
