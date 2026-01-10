import pandas as pd
import numpy as np
from config import FEATURES, WINDOW_SIZE, STEP_SIZE, LABEL_MAP

def load_csv(path):
    df = pd.read_csv(path)

    df['label'] = df['label'].map(LABEL_MAP)
    df = df.dropna()

    return df

def create_windows(df):
    X, y = [], []

    data = df[FEATURES].values
    labels = df['label'].values

    for i in range(0, len(df) - WINDOW_SIZE, STEP_SIZE):
        X.append(data[i:i + WINDOW_SIZE])
        y.append(
            np.bincount(labels[i:i + WINDOW_SIZE]).argmax()
        )

    return np.array(X), np.array(y)
