import os
import glob
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras import layers, models
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder

# ================= CONFIG =================
DATA_DIR = "../data"
WINDOW_SIZE = 50
STRIDE = 10
FEATURES = ["pitch", "roll", "svm", "gx", "gy", "gz"]
LABEL_COL = "label"

# ================= WINDOW =================
def sliding_window(data, labels, win, stride):
    X, y = [], []
    for i in range(0, len(data) - win, stride):
        X.append(data[i:i+win])
        y.append(labels[i+win-1])
    return np.array(X), np.array(y)

# ================= LOAD CSV =================
def load_all_csv(folder):
    X_all, y_all = [], []

    csv_files = glob.glob(os.path.join(folder, "*.csv"))
    print(f"📂 Found {len(csv_files)} CSV files")

    for file in csv_files:
        print(f"  → {os.path.basename(file)}")
        df = pd.read_csv(file)

        data = df[FEATURES].values.astype(np.float32)
        labels = df[LABEL_COL].values

        X, y = sliding_window(data, labels, WINDOW_SIZE, STRIDE)

        if len(X) == 0:
            continue

        X_all.append(X)
        y_all.append(y)

    return np.concatenate(X_all), np.concatenate(y_all)

# ================= NORMALIZE =================
def compute_scaler(X):
    mean = X.mean(axis=(0,1))
    std = X.std(axis=(0,1)) + 1e-6
    return mean, std

# ================= MODEL =================
def build_model(input_shape, num_classes):
    return models.Sequential([
        layers.Conv1D(16, 3, activation="relu", input_shape=input_shape),
        layers.MaxPooling1D(2),
        layers.Conv1D(32, 3, activation="relu"),
        layers.GlobalAveragePooling1D(),
        layers.Dense(32, activation="relu"),
        layers.Dense(num_classes, activation="softmax")
    ])

# ================= MAIN =================
def main():
    X, y_text = load_all_csv(DATA_DIR)

    # Encode label text → int
    le = LabelEncoder()
    y = le.fit_transform(y_text)

    print("📊 Label mapping:")
    for i, name in enumerate(le.classes_):
        print(f"  {i} → {name}")

    mean, std = compute_scaler(X)
    X = (X - mean) / std

    # save scaler + label map
    os.makedirs("../models", exist_ok=True)
    with open("../models/scaler.txt", "w") as f:
        f.write("mean:\n")
        f.write(",".join(map(str, mean)) + "\n")
        f.write("std:\n")
        f.write(",".join(map(str, std)) + "\n")

    with open("../models/labels.txt", "w") as f:
        for i, name in enumerate(le.classes_):
            f.write(f"{i},{name}\n")

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, stratify=y, random_state=42
    )

    model = build_model(X.shape[1:], len(le.classes_))
    model.compile(
        optimizer="adam",
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"]
    )

    model.fit(
        X_train, y_train,
        epochs=20,
        batch_size=32,
        validation_data=(X_test, y_test)
    )

    model.save("../models/activity_cnn.h5")
    print("✅ Model saved")

if __name__ == "__main__":
    main()
