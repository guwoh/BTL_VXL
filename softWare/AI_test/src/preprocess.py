import numpy as np
from sklearn.preprocessing import StandardScaler

def normalize(X):
    scaler = StandardScaler()

    N, T, C = X.shape
    X2 = X.reshape(-1, C)
    X2 = scaler.fit_transform(X2)
    Xn = X2.reshape(N, T, C)

    return Xn, scaler

def save_scaler(scaler, path):
    with open(path, "w") as f:
        f.write("mean:\n")
        f.write(",".join(map(str, scaler.mean_)) + "\n")
        f.write("std:\n")
        f.write(",".join(map(str, scaler.scale_)))
