# Export weights of a Keras model to C header files with int8 quantization. */
import os
import numpy as np
import tensorflow as tf

OUT_DIR = "../test"
MODEL_PATH = "../models/activity_cnn.h5"

os.makedirs(OUT_DIR, exist_ok=True)

def quantize_int8(x):
    max_val = np.max(np.abs(x))
    scale = max_val / 127.0 if max_val > 0 else 1.0
    x_q = np.round(x / scale).astype(np.int8)
    return x_q, scale

def save_conv1d(layer, name):
    W, B = layer.get_weights()
    # W shape: (K, Cin, Cout)

    Wq, w_scale = quantize_int8(W)
    Bq = np.round(B / w_scale).astype(np.int32)

    with open(f"{OUT_DIR}/{name}.h", "w") as f:
        f.write("#pragma once\n\n")
        f.write(f"#define {name.upper()}_K {W.shape[0]}\n")
        f.write(f"#define {name.upper()}_CIN {W.shape[1]}\n")
        f.write(f"#define {name.upper()}_COUT {W.shape[2]}\n")
        f.write(f"#define {name.upper()}_WSCALE {w_scale:.8e}f\n\n")

        f.write(f"static const int8_t {name}_w[{W.shape[0]}][{W.shape[1]}][{W.shape[2]}] = {{\n")
        for k in range(W.shape[0]):
            f.write("  {\n")
            for ci in range(W.shape[1]):
                row = ", ".join(str(v) for v in Wq[k, ci])
                f.write(f"    {{{row}}},\n")
            f.write("  },\n")
        f.write("};\n\n")

        f.write(f"static const int32_t {name}_b[{W.shape[2]}] = {{\n")
        f.write(", ".join(str(v) for v in Bq))
        f.write("\n};\n")

    print(f"✅ Saved {name}.h")

def save_dense(layer, name):
    W, B = layer.get_weights()
    # W shape: (Cin, Cout)

    Wq, w_scale = quantize_int8(W)
    Bq = np.round(B / w_scale).astype(np.int32)

    with open(f"{OUT_DIR}/{name}.h", "w") as f:
        f.write("#pragma once\n\n")
        f.write(f"#define {name.upper()}_CIN {W.shape[0]}\n")
        f.write(f"#define {name.upper()}_COUT {W.shape[1]}\n")
        f.write(f"#define {name.upper()}_WSCALE {w_scale:.8e}f\n\n")

        f.write(f"static const int8_t {name}_w[{W.shape[0]}][{W.shape[1]}] = {{\n")
        for ci in range(W.shape[0]):
            row = ", ".join(str(v) for v in Wq[ci])
            f.write(f"  {{{row}}},\n")
        f.write("};\n\n")

        f.write(f"static const int32_t {name}_b[{W.shape[1]}] = {{\n")
        f.write(", ".join(str(v) for v in Bq))
        f.write("\n};\n")

    print(f"✅ Saved {name}.h")

def main():
    model = tf.keras.models.load_model(MODEL_PATH)
    model.summary()

    conv_idx = 0
    dense_idx = 0

    for layer in model.layers:
        if isinstance(layer, tf.keras.layers.Conv1D):
            save_conv1d(layer, f"conv{conv_idx+1}")
            conv_idx += 1
        elif isinstance(layer, tf.keras.layers.Dense):
            save_dense(layer, f"dense{dense_idx+1}")
            dense_idx += 1

if __name__ == "__main__":
    main()
