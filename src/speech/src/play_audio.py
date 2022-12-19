#! /usr/bin/env python
from pydub import AudioSegment
from pydub.playback import play

def play_audio(wav_file):
    sound = AudioSegment.from_file(file = wav_file,format = "wav")
    play(sound)
    
def select_audio(msg: int):
    route = './src/speech/resources/speech_audio/'
    if msg==1:
        wav_file = route+'up_button.wav'
        play_audio(wav_file)
        print(f'play up_button.wav...')
    elif msg==2:
        wav_file = route+'floor_button.wav'
        # play_audio(wav_file)
        # print(f'play floor_button.wav...')
    elif msg==3:
        wav_file = route+'done.wav'
        play_audio(wav_file)
        print(f'play done.wav...')

def get_instruction():
    sound = AudioSegment.from_file(file = './src/speech/resources/speech_audio/ask.wav',format = "wav")
    play(sound)
    
def reply_instruction(target='elevator'):
    if target == 'elevator':
        sound = AudioSegment.from_file(file = './src/speech/resources/speech_audio/look_for.wav',format = "wav")
    else:
        sound = AudioSegment.from_file(file='./src/speech/resources/speech_audio/first_floor.wav', format='wav')
    play(sound)


if  __name__ == '__main__':
    while True:
        vid = int(input('Input sound to play: '))
        if vid == 1:
            get_instruction()
        elif vid == 2:
            reply_instruction()
        elif vid == 3:
            select_audio(1)
        elif vid == 4:
            select_audio(2)
        elif vid == 5:
            select_audio(3)
        else:
            pass
