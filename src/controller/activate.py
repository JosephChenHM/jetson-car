#!/usr/bin/env python
"""Create Pilot Model."""
import os
import rospy
import numpy as np
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

    # TODO: preprocess image to send to file or preprocess in higher level

    # predict output
    prediction = model.predict(image[np.newaxis, :, :, np.newaxis])
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


if __name__ == "__main__":
    package_path = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(
        package_path, "..", "..", "model",
        rospy.get_param('model_path'))
    print("Activating AutoPilot model..\n")
    pilot = Pilot(lambda: load_model(model_path), drive)
    rospy.spin()
