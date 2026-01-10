from tensorflow.keras import layers, models
from config import WINDOW_SIZE, FEATURES, NUM_CLASSES

def build_model():
    model = models.Sequential([
        layers.Input(shape=(WINDOW_SIZE, len(FEATURES))),

        layers.Conv1D(8, 5, activation='relu'),
        layers.MaxPooling1D(2),

        layers.Conv1D(16, 3, activation='relu'),
        layers.MaxPooling1D(2),

        layers.Flatten(),
        layers.Dense(16, activation='relu'),
        layers.Dense(NUM_CLASSES, activation='softmax')
    ])

    model.compile(
        optimizer='adam',
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )

    return model
