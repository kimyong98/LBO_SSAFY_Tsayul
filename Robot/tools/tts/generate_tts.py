import os
import json
from gtts import gTTS

CONST_PATH_JSON = ""
CONST_PATH_OUTPUT = ""

def generate_tts(text, lang='ko', save=None):
    tts = gTTS(text, lang=lang)
    if save:
        if os.path.isdir(save):
            output_path = os.path.join(save, "tts_sample.mp3")
        else:
            output_path = save

        if os.path.exists(os.path.pardir(output_path)) == False:
            os.makedirs(os.path.pardir(output_path))

        tts.save(output_path)

def read_json(path, enc='UTF8'):
    with open(path, 'r', encoding=enc) as f:
        return json.load(f)

def main():
    meta_dict = read_json(CONST_PATH_JSON)
    for k, v in meta_dict:
        generate_tts(v.text, save=os.path.join(CONST_PATH_OUTPUT, v.file))