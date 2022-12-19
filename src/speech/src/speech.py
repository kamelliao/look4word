import io
import logging
from typing import Dict, Union
from argparse import ArgumentParser, Namespace

import speech_recognition as sr

from mfcc import wav2mfcc
from elevator_state_cls import predict


''' use existing file '''
def speech2text_file(filename: str):
    r = sr.Recognizer()
    WAV = sr.AudioFile(filename)
    with WAV as source:
        audio = r.record(source)
    response = r.recognize_google(audio, show_all=True)
    for result in response['alternative']:
        print("Transcript: {}".format(result['transcript']))


''' use microphone '''
def speech2text(
    r: sr.Recognizer,
    microphone: sr.Microphone,
    mode: str = 'google_api',
    language: str = 'en-US'
) -> Dict[str, Union[str, float]]:
    '''
    Parameters
    ----------
    mode : str
        'google_api' | 'elevator'
    language : str
        Indicate language code if using 'google_api' for speech recognition.

    Return
    ------
    Dict[str, Union[str, float]]
        {'label': str, 'proba': float} if mode == 'elevator'
        {'transcript': str} if mode == 'google_api'
    '''

    with microphone as source:
        r.adjust_for_ambient_noise(source)

        print("Please say something")
        audio = r.listen(source)
        print("Recognizing Now .... ")

        if mode == 'elevator':
            mfcc = wav2mfcc(io.BytesIO(audio.get_wav_data()))
            mfcc = [mfcc.flatten()]

            pred_svm, prob_svm = predict(mfcc)
            return pred_svm, prob_svm

        if mode == 'google_api':
            try:
                response = r.recognize_google(audio, language=language) 
                transcript = response#['alternative'][0]['transcript']
                return transcript
            except Exception as e:
                print("Error :  " + str(e))

    return 0


def main(args: Namespace):
    r = sr.Recognizer()
    microphone = sr.Microphone(device_index=9, sample_rate=16000, chunk_size=2048)

    with microphone as source:
        r.adjust_for_ambient_noise(source)

        while True:
            print("Please say something")
            audio = r.listen(source)
            print("Recognizing Now .... ")

            if args.elevator:
                mfcc = wav2mfcc(io.BytesIO(audio.get_wav_data()))
                mfcc = [mfcc.flatten()]

                pred_svm, prob_svm = predict(mfcc)
                print(f'Prediction | label: {pred_svm:<4} | proba: {prob_svm:.3f}')

            if args.user:
                try:
                    recog = r.recognize_google(audio, language = 'en-US') 
                    print("You have said \n" + recog)
                except Exception as e:
                    print("Error :  " + str(e))

            if args.output:
                with open(args.output, "wb") as f:
                    f.write(audio.get_wav_data())
                    print(f"Audio saved to '{args.output}' successfully \n ")


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-u', '--user', action='store_true')
    parser.add_argument('-e', '--elevator', action='store_true')
    parser.add_argument('-o', '--output', type=str, default=None)
    args = parser.parse_args()

    main(args)
