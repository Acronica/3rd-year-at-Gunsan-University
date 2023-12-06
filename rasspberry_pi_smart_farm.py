from flask import Flask, render_template, redirect, Response, jsonify
from flask_crontab import Crontab
import sqlite3
import datetime as dt
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import base64, io
import serial
import cv2
import os, sys
import signal
import json

plt.rcParams['font.family'] ='NanumSquare'
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams["figure.figsize"] = [2000, 1200]
plt.rcParams["figure.dpi"] = 300

app = Flask(__name__)
crontab = Crontab(app)

@crontab.job(minute="*/5")
def db_update():
    conn = sqlite3.connect("./sensor.db")
    now = dt.datetime.now()
    data = dhtserial()
    conn.execute(f'''INSERT INTO "Measure1" VALUES (NULL,?,?,?,?)''', (data['humi'],data['temp'],data['ground_humi'],now) )
    conn.commit()
    measure1_data = conn.execute("SELECT * FROM Measure1").fetchall()
    for rows in measure1_data:
        print(rows)
    conn.close()

@crontab.job(hour="7")
def led_hour_control_on():
    GPIO.output(relay_pin, GPIO.HIGH)

@crontab.job(hour="19")
def led_hour_control_off():
    GPIO.output(relay_pin, GPIO.LOW)


def sql(name):
    conn = sqlite3.connect("sensor.db")
    data = conn.execute(f"SELECT * FROM {name}").fetchall()
    conn.close()
    return data

def plotsql_to_base64png(data):
    id, humidities, temperatures, ground_humi, timestamps = zip(*data[-51:])

    # plt.rcParams.update({
    # "figure.facecolor":  (1.0, 1.0, 1.0, 0.2),
    # "axes.facecolor":    (1.0, 1.0, 1.0, 0.4),
    # "savefig.facecolor": (1.0, 1.0, 1.0, 0.2),
    # })

    plt.figure(figsize=(10, 6), facecolor='white')
    plt.plot(id, humidities, label='습도')
    plt.plot(id, temperatures, label='온도')
    plt.plot(id, ground_humi, label='토양습도')

    plt.title('센서값')
    plt.xlabel('#')
    plt.ylabel('값')
    plt.legend()
    plt.xticks(id[::5], rotation=45)
    plt.grid()

    imgfile = io.BytesIO()
    plt.savefig(imgfile, format="png")  # 이미지 파일로 저장
    url = "data:image/png;base64,"+(base64.b64encode(imgfile.getvalue())).decode('utf-8')
    return url

def dhtserial():
    ser = serial.Serial('/dev/ttyACM0', 9600)
    try:
        data = ser.readline()
        decoded_data = data.decode('utf-8')
        parsed_values = json.loads(decoded_data)
    except:
        data = ser.readline()
        decoded_data = data.decode('utf-8')
        parsed_values = json.loads(decoded_data)
    print(decoded_data)
    ser.close()
    return parsed_values

@app.route('/')
def root():
    data = sql("Measure1")
    b64png = plotsql_to_base64png(data)
    id, humidities, temperatures, ground_humi, timestamps = zip(*data[-1:])
    real = {"ground_humi":ground_humi[0],"humi":humidities[0],"temp":temperatures[0]}
    return render_template("index.html", data=data, real=real, imgurl=b64png)

@app.route('/system/led_off')
def rodot():
    GPIO.output(relay_pin, GPIO.LOW)
    return redirect("/")

@app.route('/system/led_on')
def rodt():
    GPIO.output(relay_pin, GPIO.HIGH)
    return redirect("/")

@app.route('/system/fan_on')
def fanon():
    #GPIO.output(21, GPIO.HIGH)
    return redirect("/")

@app.route('/system/fan_off')
def fanoff():
    #GPIO.output(21, GPIO.LOW)
    return redirect("/")

@app.route('/system/heat')
def heat():
    #GPIO.output(20, GPIO.HIGH)
    return redirect("/")

@app.route('/system/cool')
def cool():
    #GPIO.output(22, GPIO.HIGH)
    return redirect("/")

@app.route('/system/alloff')
def alloff():
    #GPIO.output(20, GPIO.LOW)
    #GPIO.output(21, GPIO.LOW)
    #GPIO.output(22, GPIO.LOW)
    return redirect("/")

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(vc), mimetype='multipart/x-mixed-replace; boundary=frame')

class VideoCamera(object):
    racecontidion = False
    def __init__(self):
        if os.path.exists(".cam_using"):
            print("카메라중복생성")
            self.racecontidion = True
            return
        self.video = cv2.VideoCapture(0)
        if(self.video.read()[0] is False):
            self.video.release()
            self.video = cv2.VideoCapture(1)
        if(self.video.read()[0] is False):
            self.video.release()
            self.video = cv2.VideoCapture(2)
        if(self.video.read()[0] is False):
            self.video.release()
            self.video = cv2.VideoCapture(3)
        if self.video.read()[0] is True:
            with open('.cam_using', 'w') as file_data:
                file_data.write("cam using!")

    def __del__(self):
        if (self.racecontidion is not True) and (os.path.exists(".cam_using")):
            self.video.release()
            os.remove(".cam_using")
        elif self.racecontidion is not True:
            self.video.release()

    def get_frame(self):
        success, image = self.video.read()
        if(success is False): return b''
        _, jpeg = cv2.imencode('.jpg', image)
        return jpeg.tobytes()

vc = VideoCamera()

def signal_handler(sig, frame):
    print('SIGINT signal!')
    try:
        vc.__del__()
    except: pass
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
relay_pin = 21
GPIO.setup(21, GPIO.OUT) #one-relay
GPIO.setup(19, GPIO.OUT) #4ch-relay-first
GPIO.setup(16, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
