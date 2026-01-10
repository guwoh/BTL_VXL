import tensorflow as tf
import numpy as np

MODEL_PATH = "../models/activity_cnn.h5"
OUT_PATH   = "../models/activity_cnn_int8.tflite"

# Dummy representative data (VERY IMPORTANT)
def rep_data():
    for _ in range(100):
        yield [np.random.randn(1, 50, 6).astype(np.float32)]

model = tf.keras.models.load_model(MODEL_PATH)

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = rep_data

converter.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS_INT8
]
converter.inference_input_type  = tf.int8
converter.inference_output_type = tf.int8

tflite_model = converter.convert()

with open(OUT_PATH, "wb") as f:
    f.write(tflite_model)

print("✅ Exported INT8 TFLite model")
