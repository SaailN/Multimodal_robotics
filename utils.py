from PIL import Image

def compress_image(image_path, output_path, quality=20):
    image = Image.open(image_path)
    image.save(output_path, quality=quality)