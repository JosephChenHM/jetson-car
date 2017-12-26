import pickle
import numpy as np
import cv2
import os
import h5py
from scipy.misc import imresize
from FLAGS import *                                   # Stores parameters and hy
from keras.optimizers import Adam
from DatNet import DatNet, mse_steer_angle      # Compute the loss of steering angle
from keras.models import model_from_json
from keras.callbacks import ModelCheckpoint, EarlyStopping
import matplotlib.pyplot as plt
plt.interactive(False)

# Import data
# ########## FILE READER ##############
# #####################################
# data = pickle.load(open('../straight.p', 'rb'))
# data2 = pickle.load(open('../recovery.p', 'rb'))
DATA_PATH = os.path.join(os.environ["HOME"],"share", "dataset")

# images = data['features']
# measurements = data['labels']
# images_2 = data2['features']
# measurements_2 = data2['labels']

train_path = os.path.join(DATA_PATH, "recording1_remap.h5")
test_path = os.path.join(DATA_PATH, "Testing_remap.h5")

train_ds = h5py.File(train_path, "r")
test_ds = h5py.File(test_path, "r")

train_x = train_ds["video/image"][()].astype("float32")
train_y = train_ds["command/steering"][()].astype("float32")

test_x = test_ds["video/image"][()].astype("float32")
test_y = test_ds["command/steering"][()].astype("float32")

ds_x = np.concatenate((train_x, test_x), axis=0)
ds_y = np.concatenate((train_y, test_y), axis=0)
ds_y = ds_y[..., np.newaxis]

# images = np.concatenate((images, images_2))
# measurements = np.concatenate((measurements, measurements_2))

ds_x_new = np.zeros((ds_x.shape[0], 48, 64, 1))
for frame_idx in range(ds_x.shape[0]):
    ds_x_new[frame_idx, :, :, 0] = imresize(
        ds_x[frame_idx, :, :, 0], (48, 64))
ds_x = ds_x_new

print (ds_x.shape)

# Crop the top of the images - the sky
# images = images[:, 29:75, ...]

plt.imshow(ds_x[10])
plt.show()
# Get only a portion of the dataset
# images = images[:int(len(images)/1)]
# measurements = measurements[:int(len(measurements)/1)]
images = ds_x
measurements = ds_y
# measurements = measurements[:, :2]  # Remove speed output
print("Data loaded : Input {} // Measurement {}".format(np.shape(images), np.shape(measurements)))

# #############################
# ## DATA AUGMENTATION ########
###############################
from image_processor import random_transform
augmented_images = []
augmented_measurements = []
for image, measurement in zip(images, measurements):
    flipped_image = cv2.flip(image, 1)
    augmented_images.append(flipped_image)
    flipped_angle = measurement[0] * -1.0
    augmented_measurements.append((flipped_angle, measurement[1], measurement[2]))
    # #
    rand_image, _ = random_transform(image)
    augmented_images.append(rand_image)
    augmented_measurements.append(measurement)

# # TODO:
# How to divide this into frame block for RNN
print(np.shape(images))
print(np.shape(augmented_images))
images = np.concatenate((images, augmented_images))
measurements = np.concatenate((measurements, augmented_measurements))

# Build new model
model = DatNet(input_shape=(HEIGHT, WIDTH, CHANNELS), res_layer_params=(3, 32, 4))

model.vision_model.load_weights('./model/cnn.h5', by_name=True)
print("Pre-trained model loaded...")

model.vision_model.summary()
model.vision_model.compile(optimizer=Adam(lr=0.00002), loss=[mse_steer_angle])

checkpoint = ModelCheckpoint('./model/checkpoints/weights.{epoch:02d}-{val_loss:.3f}.h5', save_weights_only=True)
model.vision_model.fit(images, measurements, batch_size=128, epochs=1, callbacks=[checkpoint],
                       validation_split=0.3, shuffle=True)

# Save model
json_string = model.vision_model.to_json()
with open('./model/cnn.json', 'w') as outfile:
    outfile.write(json_string)
model.vision_model.save_weights('./model/cnn.h5')
print('Model saved')
# # Post-process angle
