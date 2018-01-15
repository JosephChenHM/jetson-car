#!/usr/bin/env python
"""Create Pilot Model."""
import os
import rospy
import numpy as np
from scipy.misc import imresize
from keras.models import model_from_json
from keras.backend import tf as ktf
from Pilot import Pilot


def drive(model, image):
    """ Make prediction on steering angle given an image

    # Parameters
    model : A valid Keras model
    image : numpy.ndarray
        A input image

    # Returns
    steering_angle : float
        steering angle to send
    throttle : float
        throttle value to send
    """
    if image is None:
        return

    # predict output
    prediction = model.predict(image)
    steering_angle = prediction[0][0]
    throttle = 0.1

    # TODO: POST STEER ANGLE PROCESSING - PID Controller
    return steering_angle, throttle


def load_model(model_path):
    """Load a Keras model.

    # Parameters
    model_path : str
        absolute path to the Keras model.

    # Returns
    model : A Keras model.
    """
    with open(model_path, 'r') as json_file:
        json_model = json_file.read()
        model = model_from_json(json_model, custom_objects={"ktf": ktf})
    print('Pilot model is loaded...')
    model.compile("adam", "mse")

    pre_trained_weights = model_path.replace('json', 'h5')
    model.load_weights(pre_trained_weights)

    return model


def img_preproc(aps_image, dvs_image, config=None):
    """Do custom image preprocssing here.

    # Parameters
    aps_image : numpy.ndarray
        aps image
    dvs_image : numpy.ndarray
        dvs image
    config : dict
        dictionary that contains configuration
        for the input image of the target model

    # Returns
    img : numpy.ndarray
        a 4-D tensor that is a valid input to the model.
    """
    if config is not None:
        frame_cut = config["frame_cut"]
        target_size = config["target_size"]
        clip_value = config["clip_value"]
        mode = config["mode"]
    else:
        mode = 2  # 0: DVS, 1: APS, 2: combined

    # re-normalize image
    dvs_image /= float(clip_value*2) if mode in [0, 2] else None
    aps_image /= 255. if mode in [1, 2] else None

    # cut useless content
    dvs_image = dvs_image[frame_cut[0][0]:-frame_cut[0][1],
                          frame_cut[1][0]:-frame_cut[1][1]] \
        if dvs_image is not None else None
    aps_image = aps_image[frame_cut[0][0]:-frame_cut[0][1],
                          frame_cut[1][0]:-frame_cut[1][1]] \
        if aps_image is not None else None

    # resize to target size
    dvs_image = imresize(dvs_image, target_size) if dvs_image is not None \
        else None
    aps_image = imresize(aps_image, target_size) if aps_image is not None \
        else None

    if mode == 2:
        return np.stack((dvs_image, aps_image), axis=-1)[np.newaxis, ...]
    elif mode == 0:
        return dvs_image
    elif mode == 1:
        return aps_image


if __name__ == "__main__":
    package_path = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(
        package_path, "..", "..", "model",
        rospy.get_param('model_path'))
    print("Activating AutoPilot model..\n")
    img_config = {}
    img_config["frame_cut"] = rospy.get_param("frame_cut")
    img_config["img_shape"] = tuple(rospy.get_param("img_shape"))
    img_config["target_size"] = tuple(rospy.get_param("target_size"))
    img_config["clip_value"] = rospy.get_param("clip_value")
    img_config["mode"] = rospy.get_param("mode")
    print (img_config)
    pilot = Pilot(lambda: load_model(model_path), drive,
                  img_preproc, img_config=img_config)
    rospy.spin()
