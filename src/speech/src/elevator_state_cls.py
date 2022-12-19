from joblib import dump, load
from typing import Tuple

import numpy as np
import pandas as pd
from sklearn.model_selection import cross_val_score
from sklearn.svm import SVC
# from sklearn.neighbors import KNeighborsClassifier

TRAIN_DATA_PATH = './src/speech/resources/mfcc_feats.json'
MODEL_PATH = './src/speech/resources/elevator_svm.joblib'
ELEVATOR_STATES = {
    'close': '關門中',
    'open': '開門中',
    'down': '電梯下樓',
    'up': '電梯上樓',
    'floor1': '一樓到了',
    'floor2': '二樓到了',
    'floor3': '三樓到了',
    'floor4': '四樓到了',
}


def train():
    train_data = pd.read_json(TRAIN_DATA_PATH)
    train_data = train_data[train_data['label'] != 'blank']
    train_data['mfcc'] = train_data['mfcc'].apply(lambda x: np.array(x).flatten())

    X, y = train_data['mfcc'].tolist(), train_data['label'].tolist()

    # model = KNeighborsClassifier(n_neighbors=4, metric='cosine')
    model = SVC(kernel='rbf', gamma=1e-4, C=1e2, random_state=42, probability=True)

    scores = cross_val_score(model, X, y, cv=5)
    print(f'Accuracy {scores.mean()}')

    model.fit(X, y)
    dump(model, MODEL_PATH)


def predict(X: np.ndarray) -> Tuple[str, float]:
    cls = load(MODEL_PATH)

    label = cls.predict(X).item()
    label = ELEVATOR_STATES.get(label)
    prob = cls.predict_proba(X).max()

    return label, prob


if __name__ == '__main__':
    train()