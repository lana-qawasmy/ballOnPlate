import time
import math
import cv2 as cv
import numpy as np 
import tkinter as tk
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory


factory = PiGPIOFactory()

#Center coordinates
center_x = 223
center_y = 223

prevX = 0
prevY = 0 

servo = [0,0]
#-------------------------------- Camera Part ----------------------------------------------------------------------------
#Initialize the capture 
cap = cv.VideoCapture(0)
#Finds the ball's center
def ballFinder():
    #while True:
        ret, frame = cap.read()
        result = (0,0)
        if ret:
            frame = frame[:, 93:550, :]
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            lower_value = np.array([10, 100, 120])
            higher_value = np.array([25,255,255])
            mask = cv.inRange(hsv, lower_value, higher_value)
            #mask = cv.blur(mask,(6,6))                        
            #mask = cv.erode(mask, None, iterations=2)         
            #mask = cv.dilate(mask, None, iterations=2)
            #cv.imshow("Frame", frame)
            #key = cv.waitKey(1)

            contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            #print(contours)
           # cv.imshow("ctr", mask)
            #key = cv.waitKey(1)
            i = 0 
            for cnt in contours:
                i = i + 1
                area = cv.contourArea(cnt)
                if area > 500:
                    (x,y), radius = cv.minEnclosingCircle(cnt)
                    x = int(x)
                    y = 427 - int(y) #In images, y=0 is on top, not on the bottom 
                    radius = int(radius)
                    if radius > 20:
                        result = (x,y)
                print(i)
                       # print(result)
        return(result)

#-------------------------------- Calibration Part  ----------------------------------------------------------------------------    
#This allows you to properly place the plate beneath the camera
def CenterCalibration():
    for i in np.arange(0,150):
        ret, frame = cap.read()
        if ret:
            frame = frame[:,93:550,:]
            cv.circle(frame,(center_x,center_y),3,[255,130,130],3)
            cv.imshow("Calibration", frame)
           # cv.waitKey(1)
        else:
            pass
           
        
started = False

#-------------------------------- Servo Part ----------------------------------------------------------------------------
#Initialize the two servos
servo[0] = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
servo[1] = Servo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

#Min and max angles of each motor
min_motor = np.array([0.25,0.25])
max_motor = np.array([-0.25,-0.25])

#Sets the angle of the motors to the chosen values
def set_angle():
    global initial_angle
    initial_angle[0] = float(e0.get())
    initial_angle[1] = float(e1.get())
    initialPos()

initial_angle = np.array([124,111]) #These depend on how level the surface is

#Initial position of the motors
def initialPos():

      servo[0].value = float(e0.get())
      servo[1].value = float(e1.get())


#Moves the motor according to the instructions given by the PID
def move_motors(pid):
    for i in np.arange(0,2,1):
        angle = initial_angle[i] - (pid[i]/150.0 * 20)
        print(pid[i])
#         print(angle)
        #angle = np.floor(angle)
        
        if angle > max_motor[i]:
            #print("angle limit high")
            angle = max_motor[i]
        if angle < min_motor[i]:
            #print("angle limit low")
            angle = min_motor[i]
        servo[i].value = angle 
        

#-------------------------------- PID Part ----------------------------------------------------------------------------
#GUI and looping
def start_program():
    global started,i
    set_values()
    reset_values()
    if started:
        start["text"] = "Start"
        started = False
    else:
        start["text"] = "Stop"
        started = True

errors = np.zeros(2)
pasterrors = np.zeros(2)

#Calculates the error for each line of action of each servo
def get_errors(x,y):
    global center_x, center_y, errors, pasterrors
    #print(center_x, center_y)
    pasterrors = np.copy(errors)
    errors[0] = y - center_y  
    #errors[1] = ((x-center_x) + (center_y - y))/2.0
    errors[1] = x-center_x
   
    #errors[abs(errors - pasterrors) > 40] = pasterrors[abs(errors - pasterrors) > 40]
    #print(abs(errors - pasterrors))
    #print(errors[0])
    #print(errors[1])
lastderivative = 0 
lastlastderivative = 0 
integral = 0
derivative = 0 
pid = 0

#Calculates the PID and normalizes it based on computation time 
def pid_control(compT):
    global errors, pasterrors
    global lastderivative, lastlastderivative, derivative
    global Kp, Ki, Kd, Ka
    global integral
    global lastpid
    global computation_time
    #output == Kp * error + Ki * integral + Kd * derivative 
    global pid 
    if np.all(errors > 10):
        integral += errors * 0.5 
    else:
        integral = 0
    Kd_normalized = np.copy(Kd) / (compT/np.mean(computation_time))
    Ka_normalized = np.copy(Ka) * (compT/np.mean(computation_time))
    Kp_normalized = np.copy(Kp)
    Kp_normalized[np.abs(errors) < 20] = Kp_normalized[np.abs(errors) < 20] * 0.5
    lastlastderivative = np.copy(lastderivative)
    lastderivative = np.copy(derivative)
    derivative = (errors - pasterrors)
    #print("SUM",np.sum(abs(derivative)))
    if np.sum(abs(derivative)) < 10:
        Kd_normalized = Kd_normalized *  (0.1*(np.sum(abs(derivative))))
    elif np.sum(abs(derivative)) > 20:
        Kd_normalized = Kd_normalized * 1.1
    if np.all(np.abs(derivative) < 0.5):
        Kd_normalized = 0
    lastpid = pid
    pid = Kp_normalized * errors + Ki * integral  + Kd_normalized * (((5*derivative) + (3*lastderivative) + (2*lastlastderivative))/10.0) + Ka_normalized * (((derivative - lastderivative) + (lastderivative - lastlastderivative))/2.0) 
    return pid 


Kp = np.zeros(2)
Kd = np.zeros(2)
Ka = np.zeros(2)
Ki = np.zeros(2)

#Resets the values, if the ball leaves the plate
def reset_values():
    global derivative, lastderivative, lastlastderivative, i
    global pasterrors, errors, pid
    derivative = 0
    lastderivative = 0
    lastlastderivative = 0 
    pid = 0
    i = 0
    #print('successfully reset all values')

#sets the coefficients 
def set_values():
    global Kp, Ki, Kd, Ka
    i = int(motor_selector.get())
    if i == 2:  
        Kp[:] = p_slider.get() / 10.0 
        Ki[:] = i_slider.get() / 10.0
        Kd[:] = pid_controld_slider.get()
        Ka[:] = a_slider.get()
    else:
        Kp[i] = p_slider.get() / 10.0 
        Ki[i] = i_slider.get() / 10.0
        Kd[i] = d_slider.get()
        Ka[i] = a_slider.get()
#     print(Kp, Ki, Kd, Ka)

t = 0

#-------------------------------- Ball Position Graph Part ----------------------------------------------------------------------------
#Refreshes the 'ball position' graph
def refresh(x, y):
    global t
    graphWindow.deiconify()
    graphCanvas.create_oval(x,y,x,y, fill="#b20000", width=4)
    graphCanvas.create_line(0,240,480,240, fill="#0069b5", width=2)
    graphCanvas.create_line(240,0,240,480, fill="#0069b5", width=2)
    if t >= 480:
        t = 0
        graphCanvas.delete("all")
        graphCanvas.create_line(0,240,480,240, fill="#0069b5", width=2)
        graphCanvas.create_line(240,0,240,480, fill="#0069b5", width=2)
        graphCanvas.create_oval(x,y,x+1,y, fill="#b20000")
    t += 4

computation_time = []

#-------------------------------- Main Loop Part ----------------------------------------------------------------------------
#Main loop
def main():
    global prevX, prevY, computation_time
    global center_x, center_y
    start_time = time.time()
    if started:
        x, y = ballFinder()
       # print("posiiton", x,y)
        if x != y != 0: 
            detected = True
            get_errors(x,y)
            compT = (time.time() - start_time)
            computation_time.append(compT)
            pid = pid_control(compT)  
            #print(pid[0])
            #print(compT / np.mean(computation_time))

            move_motors(pid) 
            prevX = x
            prevY = y
        else:
            initialPos()        
        refresh(x, y)
    lmain.after(1,main)

#-------------------------------- GUI Part  ----------------------------------------------------------------------------    
#GUI PART 

window = tk.Tk()
window.geometry("820x500")
window.title("PID Test")

p_slider = tk.Scale(window,  from_=0, to=15, orient="horizontal", label="Proportionnal", length=500, tickinterval=2.5, resolution=0.1)
p_slider.set(2)
p_slider.pack()
i_slider = tk.Scale(window,  from_=0, to=1, orient="horizontal", label="Integral", length=500, tickinterval=0.25, resolution=0.005)
i_slider.set(0)
i_slider.pack()
d_slider = tk.Scale(window,  from_=0, to=10, orient="horizontal", label="Derivative", length=500, tickinterval=10, resolution=0.1)
d_slider.set(6.2)
d_slider.pack()
a_slider = tk.Scale(window,  from_=0, to=10, orient="horizontal", label="Double Derivative", length=500, tickinterval=10, resolution=0.1)
a_slider.set(5.6)
a_slider.pack()

p_slider.place(x=250, y= 0)
i_slider.place(x=250, y= 100)
d_slider.place(x=250, y= 200)
a_slider.place(x=250, y= 300)

tk.Label(window, text="Motor0").place(x=00,y=20)
tk.Label(window, text="Motor1").place(x=00,y=50)
tk.Label(window, text="Motor_select").place(x=00,y=102)


lmain = tk.Label(window)
lmain.pack()

graphWindow = tk.Toplevel(window)
graphWindow.title('Ball position')
graphCanvas = tk.Canvas(graphWindow,width=480,height=480)
graphCanvas.pack()

motor0 = tk.StringVar(window)
motor1 = tk.StringVar(window)

motor_select = tk.StringVar(window)
motor0.set(str(initial_angle[0]))
motor1.set(str(initial_angle[1]))

motor_select.set("2")

e0 = tk.Spinbox(window, from_=-0.25, to=0.25,values=(-0.25, -0.15, -0.05, 0, 0.15, 0.25), command=set_angle, width=4, textvariable=motor0)
e1 = tk.Spinbox(window, from_= -0.25, to=0.25, values=(-0.25, -0.15, -0.05, 0, 0.15, 0.25), command=set_angle, width=4, textvariable=motor1)
#print(e0.get())

motor_selector = tk.Spinbox(window, from_=0, to=1, width=2, textvariable=motor_select)

motor_selector.place(x=90, y= 100)

e0.place(x=50, y=20)
e1.place(x=50, y = 50 )


start = tk.Button(window, text="Start" ,command=start_program)
start.place(x=20, y=400)
Breset = tk.Button(window, text="Reset", command=reset_values)
Breset.place(x=90, y= 400)
set = tk.Button(window, text='set values', command=set_values)
set.place(x=450, y = 400 )
    
#END OF GUI

#-------------------------------- Operation Part  ----------------------------------------------------------------------------    
#Final order of operations : 
initialPos()
#ballFinder()
CenterCalibration()
cv.destroyAllWindows()
main()
window.mainloop()


