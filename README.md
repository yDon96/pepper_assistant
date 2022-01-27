# Pepper Assistant

## Table of Contents
1. [Project Structure](##project-structure)
2. [Launch Files](##launch-files)
3. [Scripts Files](##scripts-files)
4. [NLU](##nlu)


### Project Structure
```
.
+-- launch
+-- scripts
|   +-- rasa_action.sh
|   +-- rasa_server.sh
+-- src
|   +-- models
|   +-- nlu
|   +-- utils
|   +-- config.yml
|   +-- nlu_interface.py
|   +-- nlu_server.py
|   +-- asr.py
|   +-- voice_recorder.py
|   +-- speaker_identification.py
+-- srv
|   +-- Dialogue.srv
+-- CMakeList.txt
+-- package.xml
```
### Launch Files

- <b>nlu</b>: Start terminal communication with nlu server.
- <b>reidentification</b>: Start all node relative to voice identification.
- <b>speech_recognition</b>: Start node to transcribe the voice and print it on terminal.
- <b>speech_nlu</b>: Start communication with nlu server using voice node as input.

### Scripts Files

- <b>rasa_action:</b> Bash script used to start Rasa action server.
- <b>rasa_server:</b> Bash script usef to start Rasa bot.

### NLU

