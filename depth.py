import numpy as np
import cv2


image_path = "/content/Screenshot 2024-06-13 124939.png"


color_image = cv2.imread(image_path)

if color_image is None:
    print("Error: Could not open image.")
    exit(0)

depth_image_path = (
    "/content/Screenshot 2024-06-13 124939.png"  # Path to your depth image file
)
depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

if depth_image is None:
    print("Error: Could not open depth image.")
    exit(0)

color_image = np.asanyarray(color_image)
depth_image = np.asanyarray(depth_image)

# the dimensions of the images
height, width, _ = color_image.shape

# the center of the image
center_x, center_y = width // 2, height // 2

# rectangle dimensions
rect_size = 50  # Size of the rectangle (50x50 pixels)
top_left = (center_x - rect_size // 2, center_y - rect_size // 2)
bottom_right = (center_x + rect_size // 2, center_y + rect_size // 2)


cv2.rectangle(color_image, top_left, bottom_right, (0, 255, 0), 2)

# Check if image is 0-d
if depth_image.ndim == 0:
    # Convert the image to a 1-d array
    depth_image = np.expand_dims(depth_image, axis=0)

# depth value at the center
depth_value = depth_image[center_y, center_x]
# image to a CV_8UC1 image
depth_image_gray = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)

depth_colormap = cv2.applyColorMap(depth_image_gray, cv2.COLORMAP_JET)
font = cv2.FONT_HERSHEY_SIMPLEX
text = f"Depth: {depth_value} mm"
cv2.putText(
    color_image,
    text,
    (top_left[0], top_left[1] - 10),
    font,
    0.5,
    (0, 255, 0),
    2,
    cv2.LINE_AA,
)

depth_colormap_dim = depth_colormap.shape
color_colormap_dim = color_image.shape

# If depth and color resol are different, resize color image to match depth image for display
if depth_colormap_dim != color_colormap_dim:
    resized_color_image = cv2.resize(
        color_image,
        dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
        interpolation=cv2.INTER_AREA,
    )
    images = np.hstack((resized_color_image, depth_colormap))
else:
    images = np.hstack((color_image, depth_colormap))

cv2.namedWindow("Image", cv2.WINDOW_AUTOSIZE)
cv2.imshow("Image", images)
cv2.waitKey(0)


cv2.destroyAllWindows()
