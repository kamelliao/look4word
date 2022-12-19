import re
import librosa
import numpy as np
import math
from pathlib import Path
import json
import numpy as np

# convert wav file to mfcc
def wav2mfcc(file_path, max_len=11):
    wave, sr = librosa.load(file_path, mono=True, sr=None)
    wave = wave[::3]
    mfcc = librosa.feature.mfcc(wave, sr=16000)

    # If maximum length exceeds mfcc lengths then pad the remaining ones
    if (max_len > mfcc.shape[1]):
        pad_width = max_len - mfcc.shape[1]
        mfcc = np.pad(mfcc, pad_width=((0, 0), (0, pad_width)), mode='constant')

    # Else cutoff the remaining parts
    else:
        mfcc = mfcc[:, :max_len]
    
    return mfcc

# compue cosine similarity of two vectors
def cosine_similarity(v1,v2):
    "compute cosine similarity of v1 to v2: (v1 dot v2)/{||v1||*||v2||)"
    sumxx, sumxy, sumyy = 0, 0, 0
    for i in range(len(v1)):
        x = v1[i]; y = v2[i]
        sumxx += x*x
        sumyy += y*y
        sumxy += x*y

    return sumxy/math.sqrt(sumxx*sumyy)

def get_score(wav1, wav2):
    mfcc1 = wav2mfcc(wav1)
    mfcc2 = wav2mfcc(wav2)
    mfcc1 = mfcc1.reshape(220,1)
    mfcc2 = mfcc2.reshape(220,1)
    cos_sim = cosine_similarity(mfcc1,mfcc2)
    return cos_sim

'''mfcc.json: wavfile, string, mfcc'''

def get_sample_list():
    with open('./src/mfcc.json', 'r') as f:
        sample_list = json.load(f)
    return sample_list

# rule-based method to recognize wav file and return string
def get_label(label: str, max_score: float = 1) -> str:
    if label == 'close' and max_score >= 0.925:
        string = '關門中'
    elif label == 'close_else' and max_score >= 0.92:
        string = '關門中'
    elif label == 'down' and max_score >= 0.925:
        string = '電梯下樓'
    elif label == 'down_else' and max_score >= 0.92:
        string = '電梯下樓'
    elif label == 'floor1' and max_score >= 0.93:
        string = '一樓到了'
    elif label == 'floor2' and max_score >= 0.92:
        string = '二樓到了'
    elif label == 'floor3' and max_score >= 0.93:
        string = '三樓到了'
    elif label == 'floor4' and max_score >= 0.93:
        string = '四樓到了'
    elif label == 'open' and max_score >= 0.95:
        string = '開門中'
    elif label == 'up' and max_score >= 0.93:
        string = '電梯上樓'
    else:
        string = '其他'
    
    return string

def get_inform(audio:str, sample_list):
    string = ''
    score_list = []
    target_mfcc = wav2mfcc(audio).flatten()
    for sample in sample_list:
        compare_mfcc = np.array(sample[2]).flatten()
        score = cosine_similarity(target_mfcc, compare_mfcc)
        score_list.append(score)
    max_score = max(score_list)
    idx = score_list.index(max_score)
    string = get_label(sample_list[idx][1], max_score)
        
    return string