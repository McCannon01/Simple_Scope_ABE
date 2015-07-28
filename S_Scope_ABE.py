#!/usr/bin/python3

# ===============================================
# Scope program using ABElectronics ADC Pi 8-Channel ADC
#
# Requires python smbus to be installed
# ================================================
#
# The following table describes the usage of connector P1
# GPIO is the General Purpose IO pin on the processor chip set
#
# Pin 01: 3.3V supply
# Pin 02: 5.0V supply
# Pin 03: GPIO 02  (I2C SDA)
# Pin 04: 5.0V supply
# Pin 05: GPIO 03  (I2C SCL)
# Pin 06: GND
# Pin 07: GPIO 04 Available Digital Input/Output (Possible Trigger)
# Pin 08: GPIO 14 (UART TXD)
# Pin 09: GND
# Pin 10: GPIO 15 (UART RXD)
# Pin 11: GPIO 17 Available Digital Input/Output (Possible Trigger)
# Pin 12: GPIO 18 Available Digital Input/Output (Possible Trigger)
# Pin 13: GPIO 27 Available Digital Input/Output (Possible Trigger)
# Pin 14: GND
# Pin 15: GPIO 22 Available Digital Input/Outout (Possible Trigger)
# Pin 16: GPIO 23 Available Digital Input/Output (Possible Trigger)
# Pin 17: 3.3V supply
# Pin 18: GPIO 24 Available Digital Input/Output (Possible Trigger)
# Pin 19: GPIO 10 (SPI bus master output)
# Pin 20: GND
# Pin 21: GPIO 09 (SPI bus mast in, slave out MISO)
# Pin 22: GPIO 25 Available Digital Input/Output (Possible Trigger)
# Pin 23: GPIO 11 (SPI bus serial clock SLCK)
# Pin 24: GPIO 08 (SPI bus chip select)
# Pin 25: GND
# Pin 26: GPIO 07 (SPI bus chip select)

# Bring in the Raspberry Pi's General Purpose IO (GPIO) library
import RPi.GPIO as GPIO

# Bring in the time functions library
from datetime import *
import time

# Bring in multi processing library
import multiprocessing
from multiprocessing import Process, Value, Array

# Bring the graphic window library
from tkinter import *
from tkinter import messagebox

# Bring in the AB Electronics code for the i2c interface to their ADC.
# I had to copy these files into the directory this script is in
# because I couldn't get the ABE suggested PATH to work for python3.
from ABE_ADCPi import ADCPi
from ABE_helpers import ABEHelpers

import os # In case we need it later

# Initialise the ADC device using i2c addresses 0x6a and 0x6b making
# channels 1-4 at ox6a and channels 5-8 at 0x6b.
# This means you have to change the jumpers on the board accordingly.
# On my Pi system the ABE RTC clock board is on address 0x68, which is the
# factory setting of the ADC board. The clock board can't be changed.

# Sample rates can be set for 12,14, 16 or 18 bits resolution.
# 12 = 240 samples per second (sps)
# 14 = 60 sps
# 16 = 15 sps
# 18 = 3.75 sps

# Set up the i2c communication and the ADC board using code supplied by AB Electronics
# For now, this routine uses the 240 sps mode at i2c address 0x06a and 0x06b. The Pi I'm using
# has the AB Electronics RTC at i2c address 0x68
i2c_helper = ABEHelpers()
bus = i2c_helper.get_smbus()
AD_Res=12 # A/D resolution
Adrs1=0x6a # A/D i2c bus address 1 (channels 1-4)
Adrs2=0x6b # A/D i2c bus address 2 (channels 5-8)
AD_ResX=AD_Res
AdrsX1=Adrs1
AdrsX2=Adrs2
# adc = ADCPi(bus, Adrs1, Adrs2, AD_Res) # Set bus, adrs1, adrs2, rate
adc=0 # This is a global to be used later as in the comment above
AD_Mod=0 # Later this will be a Toplevel window, but declared here to be a global later

# Set up shared memory for the A/D converter module which will run as a concurrent task
# Using the "_A" in the variable name here denotes AD channel x will be traced by scope channel "A".
# If more scope channels are added in the future, they will be "_B"... etc.
ADchannel_A = Value('i',0) # Channel AD converter is to read
ADvalue_A = Value('f',0.0) # Volts returned from ADC
ReadTime_A = Value('d',0.0) # This value can be quite large, so use a double float
AD_Bits_A = Value('i',0) # Bits of ADC resolution passed to converter module
AD_Adrs1_A = Value('i',0) # i2c address of ADC channels 1-4
AD_Adrs2_A = Value('i',0) # i2c address of ADC channels 5-8
AD_Set_A = Value('i',0) # Flag to indicate a change in res or adrs for ADC
ScopePower = Value('i',0) # Scope "power" on/off flag. 0=off, 1=on
AD_Error_A = Value('i',0) # Error encountered by A/D routine

ADchannel_A.value=1
AD_Bits_A.value=12
AD_Adrs1_A.value=Adrs1
AD_Adrs2_A.value=Adrs2
AD_Set_A.value=0
ScopePower.value=0
AD_Error_A.value=0


# Set up some universal variables
xScale_time = 1.0 # Default x axis scale to 1 second time
yScale_volts = 5.0 # Default y axis scale to 5 volts
x1 = 0
y1 = 0
x2 = 0
y2 = 0
LinePtr=0 # Pointer to the next tkinter canvas line ID.
pix_width=790 # Scope trace area width in pixels including axis label area
Max_Lines=pix_width+200 # Maximum number of trace lines (may need plenty!)
pix_height=340 # Scope trace area height in pixels including axis label area
Y_Lab_width=70 # Pixels on left trace area reserved for Y axis ticks and labels
X_Lab_height=50 # Pixels on bottom trace area reserved for X axis ticks and labels
Y_Axis_Pixels=pix_height-X_Lab_height # Number of pixels on Y trace portion of canvas
X_Axis_Pixels=pix_width-Y_Lab_width # Number of pixels on X trace portion of canvas
Xscale=float(X_Axis_Pixels/xScale_time) # Scale x axis pixels per second
Yscale=float(Y_Axis_Pixels/yScale_volts) # Scale y axis pixels per volt

FaultCode=0 # Some failure caused this suite to bomb. The reason is put in here.
            # 0 = no fault
            # 1-3 = AD routine not updating convert time. Probably not running.
TraceStartTime=0.0
StartNewTrace=True # Start up with a new trace
LastLine=False
ContinuousSweepA=True # Start up using continuous sweep
SingleSweepA=False
Sel_Chan=1 # Start up default A/D channel is 1
Time_x1=ReadTime_A.value # Needs to be double float
Time_x2=ReadTime_A.value # Needs to be double float
Volts_y1=0.0
Volts_y2=0.0
Threshold_A=5.0 # Initialize trace color threshold
TrigThresh_A=5.0 # Initialize trigger threshold
Thresh_A=False
ThreshA_On_Off=2
SweepSelectA=1
XScaleSelect=2
Trigger_Select=1
Config_Select=0
Dir_Select=0
Res_Select=0
Trigger_Sel=False
NewXscale=0
MsgCode=0
OldMsgCode=99
# Initialize the pin labels according to the pins on the connector and translate them to pin
# numbers used for the GPIO digital addressing for trigger use. Notice array element "0" is unused
# and is pointed to GPIO pin 4 in case it is accidentally referenced. Also notice element "9" will
# be used as the threshold trigger and doesn't use a pin, but is included for later code convenience.
Trig_Pin_Label=["None","Pin07","Pin11","Pin12","Pin13","Pin15","Pin16","Pin18","Pin22","Thresh"]
Trig_Pin_GPIO=[4,4,17,18,27,22,23,24,25,4]
Trig_Pin_Dir=["Input","Input","Input","Input","Input","Input","Input","Input","Input","unused"]
Trig_Pin_Res=["Pull_DN","Pull_DN","Pull_DN","Pull_DN","Pull_DN","Pull_DN","Pull_DN","Pull_DN","Pull_DN","Thresh"]
Trig_Pin_Conf=[False,False,False,False,False,False,False,False,False,True] # Note thresh config is always True
IO_Con=0 # Later this will be a Toplevel window, but declared here to be a global later

#
# Data entry pop up window dialog. The initial code fragments for the "_Query***"
# suite of classes and methods below was taken from an Internet example and then
# was modified so it would work here.
# There still may be more to do to make it more efficient but it's working fine now and
# I'm no Python/Tkinter expert.
# Floating point, integer, and string dialogs are all included whether or not they
# are used in this code suite.
#
class _QueryDialog(Toplevel):

    def __init__(self, title, prompt,
                 initialvalue=None,
                 minvalue = None, maxvalue = None,
                 parent = None):
        if not parent:
            parent = _default_root
        Toplevel.__init__(self, parent)
        self.transient(parent)
        if title:
            self.title(title)
        self.parent = parent
        self.result = None
        self.prompt   = prompt
        self.minvalue = minvalue
        self.maxvalue = maxvalue
        self.initialvalue = initialvalue
        body = Frame(self)
        self.initial_focus = self.body(body)
        body.pack(padx=5, pady=5)
        self.buttonbox()
        self.grab_set()
        if not self.initial_focus:
            self.initial_focus = self
        self.protocol("WM_DELETE_WINDOW", self.cancel)
        self.geometry("+%d+%d" % (parent.winfo_rootx()+50,
                                  parent.winfo_rooty()+50))
        self.initial_focus.focus_set()
        self.wait_window(self)

    # Set up the prompt and data entry field with the initial value
    def body(self, master):
        w = Label(master, text=self.prompt, justify=LEFT)
        w.grid(row=0, padx=5, sticky=W)
        self.entry = Entry(master, name="entry")
        self.entry.grid(row=1, padx=5, sticky=W+E)
        if self.initialvalue:
            self.entry.insert(0, self.initialvalue)
            self.entry.select_range(0, END)
        return self.entry

    # The operator has pressed "ok" or typed <return>. Validate the data.
    # This is really only needed here for floating point and integer inputs
    # because a string could be anything. The only time a min or max value is
    # checked is when one is supplied.
    def validate(self):
        try:
            result = self.getresult()
        except ValueError:
            messagebox.showwarning(
                "Illegal value",
                self.errormessage + "\nPlease try again",
                parent = self
            )
            return 0
        if self.minvalue is not None and result < self.minvalue:
            messagebox.showwarning(
                "Too small",
                "The allowed minimum value is %s. "
                "Please try again." % self.minvalue,
                parent = self
            )
            return 0
        if self.maxvalue is not None and result > self.maxvalue:
            messagebox.showwarning(
                "Too large",
                "The allowed maximum value is %s. "
                "Please try again." % self.maxvalue,
                parent = self
            )
            return 0
        self.result = result
        return 1

    # Add standard button box with "ok" and "cancel" buttons
    def buttonbox(self):
        box = Frame(self)
        w = Button(box, text="OK", width=10, command=self.ok, default=ACTIVE)
        w.pack(side=LEFT, padx=5, pady=5)
        w = Button(box, text="Cancel", width=10, command=self.cancel)
        w.pack(side=LEFT, padx=5, pady=5)
        self.bind("<Return>", self.ok)
        self.bind("<Escape>", self.cancel)
        box.pack()

    # Handler when "ok" button pressed or <return> key pressed
    def ok(self, event=None):
        if not self.validate():
            self.initial_focus.focus_set() # put focus back
            return
        self.withdraw()
        self.update_idletasks()
        self.cancel()

    # Handler when "cancel" button pressed or <escape> key pressed
    # Close query window then put focus back to the parent window
    def cancel(self, event=None):
        self.parent.focus_set()
        self.destroy()


class _QueryInteger(_QueryDialog):
    errormessage = "Not an integer."
    def getresult(self):
        return int(self.entry.get())

def askinteger(title, prompt, **kw): # Integer input box suite
    d = _QueryInteger(title, prompt, **kw)
    return d.result

class _QueryFloat(_QueryDialog):
    errormessage = "Not a floating point value."
    def getresult(self):
         return float(self.entry.get())

def askfloat(title, prompt, **kw): # Floating point input box suite
    d = _QueryFloat(title, prompt, **kw)
    return d.result

class _QueryString(_QueryDialog):
    def getresult(self):
        return self.entry.get()

def askstring(title, prompt, **kw): # String input box suite
    d = _QueryString(title, prompt, **kw)
    return d.result


# Define the ADC reader portion, which will run as a separate process by itself

def ADC_Reader_A(ADchannel_A,ADvalue_A,ReadTime_A,AD_Bits_A,AD_Adrs1_A,AD_Adrs2_A,
                 AD_Set_A,ScopePower,AD_Error_A):
    global adc
    while (True):
        # Proceed in the scope is turned on and no A/D errors are pending
        if ScopePower.value==1 and AD_Error_A.value==0:
            My_Chan=ADchannel_A.value
            if My_Chan <1 or My_Chan > 8: # Protect from receiving a bad channel
                My_Chan=1
            if AD_Set_A.value > 0: # Change the resolution and/or i2c address of the AD board
                try:
                    adc = ADCPi(bus, AD_Adrs1_A.value, AD_Adrs2_A.value, AD_Bits_A.value)
                except IOError:
                    AD_Error_A.value=1
                AD_Set_A.value = 0
        # This read is from an AB Electronics ADC Pi Plus - 8 channel  converter, but
        # with a different AD converter, change this to the appropriate method
            ADvalue_A.value=adc.read_voltage(My_Chan) # Read from the ADC channel
            ReadTime_A.value=time.time() # Get a close time stamp of the read completion

# Set up the graphic area where the trace is displayed
class Trace(Frame):
    def __init__(self,parent,**kw):
        super(Trace,self).__init__(parent,relief=SUNKEN,bd=5,padx=5,pady=5,**kw)
        # Create trace display area
        self.Screen=Canvas(self,width=pix_width+1,height=pix_height+1,bg="black")
        self.Screen.grid(row=0,column=0)

        # Draw y axis line
        LeftY=Y_Lab_width-1 # How far from the left (pixels) of the trace screen is the Y axis?
        LongY=pix_height-X_Lab_height+10 # How long is the Y axis including a tick mark below the X axis?
        Y_AxLine=self.Screen.create_line(LeftY,0,LeftY,LongY,fill="yellow",width=2)

        # Draw x axis line
        LeftX=Y_Lab_width-10 # Start the X axis line left of the Y axis to provide a tick mark.
        BotX=pix_height-X_Lab_height+2 # How far up from the bottom of the trace area is the X axis?
        X_AxLine=self.Screen.create_line(LeftX,BotX,pix_width,BotX,fill="yellow",width=2)
        # Draw the top Y axis and ending X axis tick marks
        Y_Top_Tick=self.Screen.create_line(LeftY-10,1,LeftY,1,fill="yellow",width=2)
        X_End_Tick=self.Screen.create_line(pix_width,BotX,pix_width,BotX+10,fill="yellow",width=2)

        # Create some non-visible (black) Y axis tick marks to be placed and made visible later.
        # Unused non-visible Y axis tick marks are "parked" in bottom left area on canvas.
        self.Y_Tick_1=self.Screen.create_line(LeftY-12,pix_height,LeftY-2,pix_height,fill="black",width=2)
        self.Y_Tick_2=self.Screen.create_line(LeftY-12,pix_height,LeftY-2,pix_height,fill="black",width=2)
        self.Y_Tick_3=self.Screen.create_line(LeftY-12,pix_height,LeftY-2,pix_height,fill="black",width=2)
        self.Y_Tick_4=self.Screen.create_line(LeftY-12,pix_height,LeftY-2,pix_height,fill="black",width=2)
        self.Y_Tick_5=self.Screen.create_line(LeftY-12,pix_height,LeftY-2,pix_height,fill="black",width=2)
        # Create some non-visible (black) Y axis tick labels to be placed and made visible later.
        # Unused non-visible Y axis tick labels are "parked" in bottom left area on canvas.
        self.Y_Label_0=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="yy.y",fill="black")
        self.Y_Label_1=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="yy.y",fill="black")
        self.Y_Label_2=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="yy.y",fill="black")
        self.Y_Label_3=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="yy.y",fill="black")
        self.Y_Label_4=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="yy.y",fill="black")
        self.Y_Label_5=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="yy.y",fill="black")
        
        # Create some non-visible (black) X axis tick marks to be placed and made visible later.
        # Unused non-visible X axis tick marks are "parked" in bottom left area on canvas.
        self.X_Tick_1=self.Screen.create_line(2,BotX+2,2,BotX+10,fill="black",width=2)
        self.X_Tick_2=self.Screen.create_line(2,BotX+2,2,BotX+10,fill="black",width=2)
        self.X_Tick_3=self.Screen.create_line(2,BotX+2,2,BotX+10,fill="black",width=2)
        self.X_Tick_4=self.Screen.create_line(2,BotX+2,2,BotX+10,fill="black",width=2)
        self.X_Tick_5=self.Screen.create_line(2,BotX+2,2,BotX+10,fill="black",width=2)
        # Create some non-visible (black) X axis tick labels to be placed and made visible later.
        # Unused non-visible X axis tick labels are "parked" in bottom left area on canvas.
        self.X_Label_0=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="xx.x",fill="black")
        self.X_Label_1=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="xx.x",fill="black")
        self.X_Label_2=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="xx.x",fill="black")
        self.X_Label_3=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="xx.x",fill="black")
        self.X_Label_4=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="xx.x",fill="black")
        self.X_Label_5=self.Screen.create_text(Y_Lab_width/2,pix_height-15,text="xx.x",fill="black")
        
        Y_Axis_Name=self.Screen.create_text(8,(pix_height/2)-(X_Lab_height/2),
                                            text="V\nO\nL\nT\nS",fill="yellow")
        X_Axis_Name=self.Screen.create_text((pix_width/2)+(Y_Lab_width/2),pix_height-8,
                                            text="SECONDS",fill="yellow")
        
        self.Lines=[]
        # Create huge line array. More than the number of pixels wide would be nice.
        # We will be changing their position and color later when data is plotted.
        # NOTE: A very fast Pi and very fast AD converter will chew these up fast, so
        # more may be needed if that is the case.
        for i in range(Max_Lines):
            self.Lines.append(self.Screen.create_line(0,0,0,0,fill="black"))
        self.Five_Volt_Y() #Default. Put ticks and labels on Y axis
        self.One_Second_XD() #Default. Put ticks and labels on X axis

    # Set up the Y axis ticks and labels for a 5 volt range, 1 volt per tick. As of this
    # writing, since the A/D converter is only 0-5v, for now this is the only y needed.
    def Five_Volt_Y(self):
        
        global Y_Axis_Pixels,Y_Lab_width
        
        Y_PixPerTick=Y_Axis_Pixels/5
        TempX1=Y_Lab_width-10

        # Y axis tick zero is part of the X axis line. Just label it.
        self.Screen.coords(self.Y_Label_0,TempX1-20,Y_Axis_Pixels)
        self.Screen.itemconfig(self.Y_Label_0,text="0.0",fill="yellow")
        
        TempY1=Y_Axis_Pixels-Y_PixPerTick
        self.Screen.coords(self.Y_Tick_1,TempX1,TempY1,Y_Lab_width,TempY1)
        self.Screen.itemconfig(self.Y_Tick_1,fill="yellow")
        self.Screen.coords(self.Y_Label_1,TempX1-20,TempY1)
        self.Screen.itemconfig(self.Y_Label_1,text="1.0",fill="yellow")
        
        TempY1=Y_Axis_Pixels-(Y_PixPerTick*2)
        self.Screen.coords(self.Y_Tick_2,TempX1,TempY1,Y_Lab_width,TempY1)
        self.Screen.itemconfig(self.Y_Tick_2,fill="yellow")
        self.Screen.coords(self.Y_Label_2,TempX1-20,TempY1)
        self.Screen.itemconfig(self.Y_Label_2,text="2.0",fill="yellow")
        
        TempY1=Y_Axis_Pixels-(Y_PixPerTick*3)
        self.Screen.coords(self.Y_Tick_3,TempX1,TempY1,Y_Lab_width,TempY1)
        self.Screen.itemconfig(self.Y_Tick_3,fill="yellow")
        self.Screen.coords(self.Y_Label_3,TempX1-20,TempY1)
        self.Screen.itemconfig(self.Y_Label_3,text="3.0",fill="yellow")
        
        TempY1=Y_Axis_Pixels-(Y_PixPerTick*4)
        self.Screen.coords(self.Y_Tick_4,TempX1,TempY1,Y_Lab_width,TempY1)
        self.Screen.itemconfig(self.Y_Tick_4,fill="yellow")
        self.Screen.coords(self.Y_Label_4,TempX1-20,TempY1)
        self.Screen.itemconfig(self.Y_Label_4,text="4.0",fill="yellow")

        TempY1=1 # No need to draw tick 5. It's the top tick.
        self.Screen.coords(self.Y_Label_5,TempX1-20,TempY1+8)
        self.Screen.itemconfig(self.Y_Label_5,text="5.0",fill="yellow")

    # Set up the X axis ticks and labels for a 1 second range, .25 secs per tick.
    # This is the default time and sets up the tick marks on the axis. Subsequent
    # calls or other selected time scales may not need to place the tick marks.
    def One_Second_XD(self):
        
        global X_Axis_Pixels,Y_Axis_Pixels,Y_Lab_width
        
        X_PixPerTick=X_Axis_Pixels/4
        TempY2=Y_Axis_Pixels+12

        # Y axis tick zero is part of the X axis line. Just label it.
        self.Screen.coords(self.X_Label_0,Y_Lab_width-1,TempY2+10)
        self.Screen.itemconfig(self.X_Label_0,text="0.0",fill="yellow")
        
        TempX1=Y_Lab_width+X_PixPerTick
        self.Screen.coords(self.X_Tick_1,TempX1,Y_Axis_Pixels+1,TempX1,TempY2)
        self.Screen.itemconfig(self.X_Tick_1,fill="yellow")
        self.Screen.coords(self.X_Label_1,TempX1,TempY2+10)
        self.Screen.itemconfig(self.X_Label_1,text="0.25",fill="yellow")

        TempX1=Y_Lab_width+(X_PixPerTick*2)
        self.Screen.coords(self.X_Tick_2,TempX1,Y_Axis_Pixels+1,TempX1,TempY2)
        self.Screen.itemconfig(self.X_Tick_2,fill="yellow")
        self.Screen.coords(self.X_Label_2,TempX1,TempY2+10)
        self.Screen.itemconfig(self.X_Label_2,text="0.5",fill="yellow")

        TempX1=Y_Lab_width+(X_PixPerTick*3)
        self.Screen.coords(self.X_Tick_3,TempX1,Y_Axis_Pixels+1,TempX1,TempY2)
        self.Screen.itemconfig(self.X_Tick_3,fill="yellow")
        self.Screen.coords(self.X_Label_3,TempX1,TempY2+10)
        self.Screen.itemconfig(self.X_Label_3,text="0.75",fill="yellow")

        TempX1=pix_width-15 # Last X tick is already on the edge. Place label accordingly.
        self.Screen.coords(self.X_Label_4,TempX1,TempY2+10)
        self.Screen.itemconfig(self.X_Label_4,text="1.0",fill="yellow")

# The half, one, two, three, four second scales are all that's needed for now. Perhaps
# in the future more scales will be added or even a scale slider. Also, for now, all
# the scales start at 0.0, so we don't need to redraw the 0.0 label.

    def Half_Second_X(self):

        #self.Screen.itemconfig(self.X_Label_0,text="0.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_1,text="0.125",fill="yellow")
        self.Screen.itemconfig(self.X_Label_2,text="0.25",fill="yellow")
        self.Screen.itemconfig(self.X_Label_3,text="0.375",fill="yellow")
        self.Screen.itemconfig(self.X_Label_4,text="0.5",fill="yellow")

    def One_Second_X(self):
        
        #self.Screen.itemconfig(self.X_Label_0,text="0.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_1,text="0.25",fill="yellow")
        self.Screen.itemconfig(self.X_Label_2,text="0.5",fill="yellow")
        self.Screen.itemconfig(self.X_Label_3,text="0.75",fill="yellow")
        self.Screen.itemconfig(self.X_Label_4,text="1.0",fill="yellow")

    def Two_Second_X(self):

        #self.Screen.itemconfig(self.X_Label_0,text="0.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_1,text="0.5",fill="yellow")
        self.Screen.itemconfig(self.X_Label_2,text="1.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_3,text="1.5",fill="yellow")
        self.Screen.itemconfig(self.X_Label_4,text="2.0",fill="yellow")

    def Three_Second_X(self):

        #self.Screen.itemconfig(self.X_Label_0,text="0.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_1,text="0.75",fill="yellow")
        self.Screen.itemconfig(self.X_Label_2,text="1.5",fill="yellow")
        self.Screen.itemconfig(self.X_Label_3,text="2.25",fill="yellow")
        self.Screen.itemconfig(self.X_Label_4,text="3.0",fill="yellow")

    def Four_Second_X(self):
        
        #self.Screen.itemconfig(self.X_Label_0,text="0.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_1,text="1.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_2,text="2.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_3,text="3.0",fill="yellow")
        self.Screen.itemconfig(self.X_Label_4,text="4.0",fill="yellow")

    # Start a new trace at x=Y_Lab_width+1 pixel. Any line object needs starting and ending
    # coordinates. The first line location is determined here. Subsequent lines will use the ending
    # location of the previous line as its first coordinate and its end coordinate will be determined
    # elsewhere.
    def New_Trace(self):
        global x1,y1,x2,y2
        global LinePtr
        global Y_Lab_width,X_Lab_height
        global Y_Axis_Pixels,X_Axis_Pixels
        global Xscale,Yscale
        global Volts_y1,Volts_y2
        global FaultCode
        global Time_x1,Time_x2,TraceStartTime
        for ii in range(LinePtr+2): # Blank out the previous trace
            self.Screen.coords(self.Lines[ii],0,0,0,0)
            self.Screen.itemconfig(self.Lines[ii],fill="black")
        Time_x1=ReadTime_A.value # If AD routine is running ReadTime_x.value will change
        TryNumber=0
        FirstPointTest=False
        # Spin around here waiting for the read time to change. This means the AD converter
        # module, which is concurrently running with this code, has actually got a reading
        # for the first point.
        while not FirstPointTest:
            if Time_x1 == ReadTime_A.value:
                time.sleep(0.001) # Wait a millisecond and go read again
                TryNumber += 1
                if TryNumber > 1000: # 1000 tries is 1 second. AD should be faster than that
                    FaultCode=1 # AD program isn't running or updating time for some reason
                    break
            else:
                TraceStartTime=ReadTime_A.value # Time stamp of first pixel of trace (x = 0)
                Time_x1=TraceStartTime
                Volts_y1=ADvalue_A.value
                FirstPointTest=True
        # It takes two points to draw a line. Spin here to get the second point when the read
        # time changes again.
        Time_x2=ReadTime_A.value
        TryNumber=0
        SecondPointTest=False
        while FirstPointTest and (not SecondPointTest):
            if Time_x2 == ReadTime_A.value:
                time.sleep(0.001) # Wait a millisecond and go read again
                TryNumber += 1
                if TryNumber > 1000: # 1000 tries is 1 second. AD should be faster than that
                    FaultCode=2 # AD program isn't running or updating time for some reason
                    break
            else:
                Time_x2=ReadTime_A.value
                Volts_y2=ADvalue_A.value
                SecondPointTest=True

        if SecondPointTest:
            # Convert time and volts to pixel coordinates
            # Place x1
            x1=Y_Lab_width+1
            # Place y1
            y1=round(Volts_y1 * Yscale)
            if y1 < 1:
                y1 = 1 # Keep over voltage on the screen (top of Y axis)
            if y1 > Y_Axis_Pixels:
                y1 = Y_Axis_Pixels # Keep under voltage on the screen (bottom of Y axis)
            y1=pix_height-X_Lab_height-y1 #Invert y-axis location (0 is top pixel on a canvas)
            # Place x2
            x2=round((Time_x2-TraceStartTime)* Xscale)+Y_Lab_width
            if x2 >= pix_width:
                x2 = pix_width
                LastLine=True
            # Place y2
            y2=round(Volts_y2 * Yscale)
            if y2 < 1:
                y2 = 1 # Keep over voltage on the screen (top of Y axis)
            if y2 > Y_Axis_Pixels:
                y2 = Y_Axis_Pixels # Keep under voltage on the screen (bottom of Y axis)
            y2=Y_Axis_Pixels-y2 #Invert y-axis location (0 is top pixel on a canvas)

            LinePtr=0 # Point to first line in huge line array
            Color_Me = "green"
            if Thresh_A and ((Volts_y1 >= Threshold_A) or (Volts_y2 >= Threshold_A)):
                Color_Me= "red"
            self.Draw_Line(Color_Me)

    def Draw_Line(self,ThisColor):
        global x1,y1,x2,y2
        global LinePtr
        global StartNewTrace
        global LastLine
        global SingleSweepA

        self.Screen.coords(self.Lines[LinePtr],x1,y1,x2,y2)#Place line
        self.Screen.itemconfig(self.Lines[LinePtr],fill=ThisColor) # Color the line
        # If this is the last line for this sweep, set up for new sweep
        if LastLine:
            # Diagnostic print showing how many lines in this trace and last line coordinates.
            # Comment out when not needed.
            #print("Last line LinePtr =",LinePtr," X1=",x1," Y1=",y1," X2=",x2," Y2=",y2)
            StartNewTrace=True
            LastLine=False
            SingleSweepA=False # This is the end of a single sweep
        
# Set up display section for threshold setting and on/off selection
# Note: This threshold for line display color, not trigger.
class Threshold_Set_A(LabelFrame):
    
    def __init__(self,parent,**kw):
        
        super(Threshold_Set_A,self).__init__(parent,relief=GROOVE,bd=5,padx=2,pady=2,
                                          fg="black",text="Threshold Check",**kw)
        # Set up Threshold A value display in a label and it's mod button
        self.Thresh_A_Label=Label(self,text=str(Threshold_A),bd=5,width=5,anchor=E,relief=RIDGE,bg="yellow")
        self.Thresh_A_Label.grid(row=0,column=0)
        self.ThreshBtnA=Button(self,bd=5,text="Mod",relief=RAISED,width=3,padx=5,pady=0,command=self.Thresh_A_Mod)
        self.ThreshBtnA.grid(row=0, column=1, sticky=N)
        self.ThreshA_ON_btn=Radiobutton(self,bd=5,indicatoron=0,variable=ThreshA_On_Off,value=1,width=5,
                                    selectcolor="green",text="On",command=self.ThreshA_ON)
        self.ThreshA_OFF_btn=Radiobutton(self,bd=5,indicatoron=0,variable=ThreshA_On_Off,value=2,width=5,
                                    selectcolor="green",text="Off",command=self.ThreshA_OFF)

        self.ThreshA_ON_btn.grid(row=1,column=0)
        self.ThreshA_OFF_btn.grid(row=1,column=1)
        
        ThreshA_On_Off.set(2) # On startup set button default to 2 (Off)

    def Thresh_A_Mod(self):
        global Threshold_A
        Results=askfloat("Floating Point","Color Theshold (Volts)",parent=self,\
                            initialvalue=Threshold_A,minvalue=0.0,maxvalue=5.0)
        if Results != None:
            Threshold_A = Results
            self.Thresh_A_Label.config(text=str(Threshold_A),width=5,anchor=E,bg="yellow")

    def ThreshA_ON(self):
        global Thresh_A
        Thresh_A = True

    def ThreshA_OFF(self):
        global Thresh_A
        Thresh_A = False

# Set up display section for sweep selection
class Sweep_Select_A(LabelFrame):
    
    def __init__(self,parent,**kw):
        
        super(Sweep_Select_A,self).__init__(parent,relief=GROOVE,bd=5,padx=2,pady=2,
                                          fg="black",text="Sweep Select",**kw)

        self.SweepA_CONT_btn=Radiobutton(self,bd=5,indicatoron=0,variable=SweepSelectA,value=1,width=5,
                                    selectcolor="green",text="Cont",command=self.SweepA_CONT)
        self.SweepA_SING_btn=Radiobutton(self,bd=5,indicatoron=0,variable=SweepSelectA,value=2,width=5,
                                    selectcolor="green",text="Sing",command=self.SweepA_SING)
        self.SweepA_NOW_btn=Button(self,bd=5,text="Man Trig",relief=RAISED,width=8,padx=4,pady=2,command=self.SweepA_NOW)

        self.SweepA_CONT_btn.grid(row=0,column=0)
        self.SweepA_SING_btn.grid(row=0,column=1)
        self.SweepA_NOW_btn.grid(row=1,column=0,columnspan=2)
        
        SweepSelectA.set(1) # On startup set button default to 1 (Continuous)

    def SweepA_CONT(self):
        global ContinuousSweepA,SingleSweepA,Trigger_Sel
        if not Trigger_Sel: # Do not allow auto continuous trace if a trigger has been selected
            ContinuousSweepA=True
            SingleSweepA=False
        else:
            SweepSelectA.set(2)

    def SweepA_SING(self):
        global ContinuousSweepA,SingleSweepA
        ContinuousSweepA=False
        SingleSweepA=True

    def SweepA_NOW(self):
        global ContinuousSweepA,SingleSweepA
        if SweepSelectA.get()==2:
            ContinuousSweepA=False
            SingleSweepA=True

# Set up display section for trigger selection
class Trigger_Sel_A(LabelFrame):
    
    def __init__(self,parent,**kw):
        
        super(Trigger_Sel_A,self).__init__(parent,relief=GROOVE,bd=5,padx=2,pady=2,
                                          fg="black",text="Trigger Select",**kw)
        self.Trig_PIN_btn=[]
        self.Config_PIN_btn=[]
        for i in range(10):
            self.Trig_PIN_btn.append(Radiobutton(self,bd=5,indicatoron=0,variable=Trigger_Select,value=i,width=5,
                                    selectcolor="green",text=Trig_Pin_Label[i],command=self.Trig_Exec))
            self.Config_PIN_btn.append(Radiobutton(self,bd=5,indicatoron=0,variable=Config_Select,value=i,width=5,
                                    selectcolor="yellow",text="Confg",fg="red",command=self.Config_Exec))  

        # Set up trigger threshold A value display in a label and it's modification button
        self.TrigThresh_A_Label=Label(self,text=str(TrigThresh_A),bd=5,width=5,anchor=E,relief=RIDGE,bg="yellow")
        self.TrigThreshBtnA=Button(self,bd=5,text="Mod",relief=RAISED,width=3,padx=5,pady=0,command=self.TrigThresh_A_Mod)

        # Note: Trig_PIN_btn[0] is actually the None (no trigger) button
        self.Trig_PIN_btn[0].grid(row=0,column=0,columnspan=2)
        self.Trig_PIN_btn[1].grid(row=1,column=0)
        self.Trig_PIN_btn[2].grid(row=2,column=0)
        self.Trig_PIN_btn[3].grid(row=3,column=0)
        self.Trig_PIN_btn[4].grid(row=4,column=0)
        self.Trig_PIN_btn[5].grid(row=5,column=0)
        self.Trig_PIN_btn[6].grid(row=6,column=0)
        self.Trig_PIN_btn[7].grid(row=7,column=0)
        self.Trig_PIN_btn[8].grid(row=8,column=0)

        # Note: IO Pin configurartion buttons 0 and 9 are meaningless and not displayed.
        self.Config_PIN_btn[1].grid(row=1,column=1)
        self.Config_PIN_btn[2].grid(row=2,column=1)
        self.Config_PIN_btn[3].grid(row=3,column=1)
        self.Config_PIN_btn[4].grid(row=4,column=1)
        self.Config_PIN_btn[5].grid(row=5,column=1)
        self.Config_PIN_btn[6].grid(row=6,column=1)
        self.Config_PIN_btn[7].grid(row=7,column=1)
        self.Config_PIN_btn[8].grid(row=8,column=1)

        # Note: Trig_PIN_btn[9] is actually the Thresh trigger button
        self.Trig_PIN_btn[9].grid(row=9,column=0,columnspan=2)
        self.TrigThresh_A_Label.grid(row=10,column=0)
        self.TrigThreshBtnA.grid(row=10, column=1)
        
        Trigger_Select.set(0) # On startup set button default to 0 (None)
        Config_Select.set(0) # On start up no config buttons are selected

# Pin configuration selected
    def Config_Exec(self):
        global IO_Con,Trig_Pin_Label,Trig_Pin_Res,Trig_Pin_Dir
        global Res_Select,Dir_Select
        self.n=Config_Select.get()
        self.Temp_Res=Trig_Pin_Res[self.n]
        self.Temp_Dir=Trig_Pin_Dir[self.n]
        Res_Select.set(0)
        if self.Temp_Res == "Pull_UP":
            Res_Select.set(1)
        Dir_Select.set(0)
        if self.Temp_Dir == "Output":
            Dir_Select.set(1)
        IO_Con=Toplevel()
        IO_Con.geometry("350x350")
        IO_Con.title("I/O Port Configure: "+Trig_Pin_Label[self.n])
        IO_Con.protocol("WM_DELETE_WINDOW", self.Con_destroy)
        Label(IO_Con,text="Press Cancel to close without changes.").grid(row=0,column=0,columnspan=3,sticky=W)
        Label(IO_Con,text="Press Apply to configure and initialize pin.").grid(row=1,column=0,columnspan=3,sticky=W)
        Label(IO_Con,text=" ").grid(row=2,column=0) # Blank line for spacing
        
        ResistorDN_btn=Radiobutton(IO_Con,bd=5,indicatoron=0,variable=Res_Select,value=0,width=7,
                                    selectcolor="green",text="Pull_DN",command=self.Pull_UP_DN)
        ResistorUP_btn=Radiobutton(IO_Con,bd=5,indicatoron=0,variable=Res_Select,value=1,width=7,
                                    selectcolor="green",text="Pull_UP",command=self.Pull_UP_DN)

        ResistorDN_btn.grid(row=3,column=1)
        ResistorUP_btn.grid(row=4,column=1)
        Label(IO_Con,text=" ").grid(row=5,column=0) # Blank line for spacing
        
        DirectionIN_btn=Radiobutton(IO_Con,bd=5,indicatoron=0,variable=Dir_Select,value=0,width=7,
                                    selectcolor="green",text="Input",command=self.Dir_IN_OUT)
        DirectionOUT_btn=Radiobutton(IO_Con,bd=5,indicatoron=0,variable=Dir_Select,value=1,width=7,
                                    selectcolor="green",text="Output",command=self.Dir_IN_OUT)
        
        DirectionIN_btn.grid(row=6,column=1)
        DirectionOUT_btn.grid(row=7,column=1)
        Label(IO_Con,text=" ").grid(row=8,column=0) # Blank line for spacing
        
        Con_Destroy_btn=Button(IO_Con,bd=2,text="Abort",relief=RAISED,command=self.Con_destroy)
        Con_Destroy_btn.grid(row=9, column=0)

        Con_Apply_btn=Button(IO_Con,bd=2,text="Apply",relief=RAISED,command=self.Con_Apply_Mod)
        Con_Apply_btn.grid(row=9, column=2)

    def Pull_UP_DN(self):
        self.Temp_Res="Pull_DN"
        if Res_Select.get() == 1:
            self.Temp_Res="Pull_UP"

    def Dir_IN_OUT(self):
        self.Temp_Dir="Input"
        if Dir_Select.get() == 1:
            self.Temp_Dir="Output"

    def Con_Apply_Mod(self):
        global Trig_Pin_Res,Trig_pin_Dir,Trig_Pin_Conf,Trig_Pin_GPIO
        
        Trig_Pin_Res[self.n]=self.Temp_Res
        Trig_Pin_Dir[self.n]=self.Temp_Dir

        if (Trig_Pin_Dir[self.n] == "Input"):
            # Select pull up or pull down resistor
            if (Trig_Pin_Res[self.n] == "Pull_UP"):
                GPIO.setup(Trig_Pin_GPIO[self.n],GPIO.IN,pull_up_down=GPIO.PUD_UP)
            else:
                GPIO.setup(Trig_Pin_GPIO[self.n],GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
        else: # If not an Input, must be Output.
            GPIO.setup(Trig_Pin_GPIO[self.n],GPIO.OUT) # Set up pin as output

        # Change config text to green to indicate it is configured
        self.Config_PIN_btn[self.n].config(fg="green")
        Trig_Pin_Conf[self.n]=True
#        print("Config done. ",Trig_Pin_Label[self.n]," GPIO=",Trig_Pin_GPIO[self.n]," ",
#              Trig_Pin_Dir[self.n]," ",Trig_Pin_Res[self.n])
        self.Con_destroy()

    def Con_destroy(self):
        Config_Select.set(0) # Release the config button selection
        IO_Con.destroy()

        
# Trigger selected. The specific trigger selected will be contained in the Radiobutton
# variable "Trigger_Select"
    def Trig_Exec(self):
        global Trigger_Sel,SweepSelectA,ContinuousSweepA,SingleSweepA,MsgCode
        self.n=Trigger_Select.get()
        if self.n == 0:
            # Trigger selected is "None"
#            print("No trigger selected")
            Trigger_Sel=False
            ContinuousSweepA=True
            SingleSweepA=False
            SweepSelectA.set(1)
        else:
            if Trig_Pin_Conf[self.n]: # Test if this trigger has been configured.
                Trigger_Sel = True
            # If the scope is in continuous sweep mode, turn it off, but switch to single sweep
            # mode so the current trace completes before testing for trigger
                if ContinuousSweepA:
                    SingleSweepA=True
                    ContinuousSweepA=False # Make sure continuous sweep is turned off for trigger mode
                SweepSelectA.set(2) # Set sweep buttons to single mode when trigger activated
            else:
                MsgCode=3
                Trigger_Select.set(0)
                Trigger_Sel=False
            
# Modify the trigger threshold value. Note: This is not the line display color threshold.
    def TrigThresh_A_Mod(self):
        global TrigThresh_A
        Results=askfloat("Floating Point","Trigger Theshold (Volts)",parent=self,\
                            initialvalue=TrigThresh_A,minvalue=0.0,maxvalue=5.0)
        if Results != None:
            TrigThresh_A = Results
            self.TrigThresh_A_Label.config(text=str(TrigThresh_A),width=5,anchor=E,bg="yellow")

# Set up display section for sweep time selection
class XScale_Select(LabelFrame):
    
    def __init__(self,parent,**kw):
        
        super(XScale_Select,self).__init__(parent,relief=GROOVE,bd=5,padx=2,pady=2,
                                          fg="black",text="X Scale Select",**kw)

        self.XScale_HALF_btn=Radiobutton(self,bd=5,indicatoron=0,variable=XScaleSelect,value=1,width=5,
                                    selectcolor="green",text="0.5",command=self.XScale_HALF)
        self.XScale_ONE_btn=Radiobutton(self,bd=5,indicatoron=0,variable=XScaleSelect,value=2,width=5,
                                    selectcolor="green",text="1.0",command=self.XScale_ONE)
        self.XScale_TWO_btn=Radiobutton(self,bd=5,indicatoron=0,variable=XScaleSelect,value=3,width=5,
                                    selectcolor="green",text="2.0",command=self.XScale_TWO)
        self.XScale_THREE_btn=Radiobutton(self,bd=5,indicatoron=0,variable=XScaleSelect,value=4,width=5,
                                    selectcolor="green",text="3.0",command=self.XScale_THREE)
        self.XScale_FOUR_btn=Radiobutton(self,bd=5,indicatoron=0,variable=XScaleSelect,value=5,width=5,
                                    selectcolor="green",text="4.0",command=self.XScale_FOUR)
        
        self.XScale_HALF_btn.grid(row=0,column=0)
        self.XScale_ONE_btn.grid(row=0,column=1)
        self.XScale_TWO_btn.grid(row=0,column=2)
        self.XScale_THREE_btn.grid(row=0,column=3)
        self.XScale_FOUR_btn.grid(row=0,column=4)
        
        XScaleSelect.set(2) # On startup set button default to 2 (1 second)

    def XScale_HALF(self):
        global xScale_time,Xscale,X_Axis_Pixels,NewXscale
        xScale_time = 0.5
        Xscale=X_Axis_Pixels/xScale_time # Scale x axis pixels per second
        NewXscale=1

    def XScale_ONE(self):
        global xScale_time,Xscale,X_Axis_Pixels,NewXscale
        xScale_time = 1.0
        Xscale=X_Axis_Pixels/xScale_time # Scale x axis pixels per second
        NewXscale=2
        
    def XScale_TWO(self):
        global xScale_time,Xscale,X_Axis_Pixels,NewXscale
        xScale_time = 2.0
        Xscale=X_Axis_Pixels/xScale_time # Scale x axis pixels per second
        NewXscale=3

    def XScale_THREE(self):
        global xScale_time,Xscale,X_Axis_Pixels,NewXscale
        xScale_time = 3.0
        Xscale=X_Axis_Pixels/xScale_time # Scale x axis pixels per second
        NewXscale=4

    def XScale_FOUR(self):
        global xScale_time,Xscale,X_Axis_Pixels,NewXscale
        xScale_time = 4.0
        Xscale=X_Axis_Pixels/xScale_time # Scale x axis pixels per second
        NewXscale=5
        
# Set up selector buttons for A/D channel choice
class AD_Channel_Sel(LabelFrame):
    
    def __init__(self,parent,**kw):
        
        super(AD_Channel_Sel,self).__init__(parent,relief=GROOVE,bd=5,padx=2,pady=2,
                                          fg="black",text="A/D Channel Select",**kw)

        self.Chan_1_btn=Radiobutton(self,bd=5,indicatoron=0,variable=Sel_Chan,value=1,width=3,
                                    selectcolor="green",text="1",command=self.New_Channel)
        self.Chan_2_btn=Radiobutton(self,bd=5,indicatoron=0,variable=Sel_Chan,value=2,width=3,
                                    selectcolor="green",text="2",command=self.New_Channel)
        self.Chan_3_btn=Radiobutton(self,bd=5,indicatoron=0,variable=Sel_Chan,value=3,width=3,
                                    selectcolor="green",text="3",command=self.New_Channel)
        self.Chan_4_btn=Radiobutton(self,bd=5,indicatoron=0,variable=Sel_Chan,value=4,width=3,
                                    selectcolor="green",text="4",command=self.New_Channel)
        self.Chan_5_btn=Radiobutton(self,bd=5,indicatoron=0,variable=Sel_Chan,value=5,width=3,
                                    selectcolor="green",text="5",command=self.New_Channel)
        self.Chan_6_btn=Radiobutton(self,bd=5,indicatoron=0,variable=Sel_Chan,value=6,width=3,
                                    selectcolor="green",text="6",command=self.New_Channel)
        self.Chan_7_btn=Radiobutton(self,bd=5,indicatoron=0,variable=Sel_Chan,value=7,width=3,
                                    selectcolor="green",text="7",command=self.New_Channel)
        self.Chan_8_btn=Radiobutton(self,bd=5,indicatoron=0,variable=Sel_Chan,value=8,width=3,
                                    selectcolor="green",text="8",command=self.New_Channel)
        self.Chan_1_btn.grid(row=0,column=0)
        self.Chan_2_btn.grid(row=0,column=1)
        self.Chan_3_btn.grid(row=0,column=2)
        self.Chan_4_btn.grid(row=0,column=3)
        self.Chan_5_btn.grid(row=0,column=4)
        self.Chan_6_btn.grid(row=0,column=5)
        self.Chan_7_btn.grid(row=0,column=6)
        self.Chan_8_btn.grid(row=0,column=7)
        Sel_Chan.set(1) # On startup set button default to channel 1

    def New_Channel(self):
        global ADchannel_A
        ADchannel_A.value=Sel_Chan.get()
        #print("New channel selected =",ADchannel_A.value)

# Set up the AD resolution and i2c address settings display and modify button
class AD_Settings(LabelFrame):
    def __init__(self,parent,**kw):
        
        super(AD_Settings,self).__init__(parent,relief=GROOVE,bd=5,padx=2,pady=2,
                                          fg="black",text="A/D Settings",**kw)
        Label(self,text="Res:").grid(row=0,column=0)
        self.Bit_Res=Label(self,text=str(AD_Res),bd=5,width=3,anchor=E,relief=RIDGE,bg="yellow")
        self.Bit_Res.grid(row=0,column=1)
        Label(self,text=" Adrs1:").grid(row=0,column=2)
        self.BusADR1=Label(self,text=str(Adrs1),bd=5,width=3,anchor=E,relief=RIDGE,bg="yellow")
        self.BusADR1.grid(row=0,column=3)
        Label(self,text=" Adrs2:").grid(row=0,column=4)
        self.BusADR2=Label(self,text=str(Adrs2),bd=5,width=3,anchor=E,relief=RIDGE,bg="yellow")
        self.BusADR2.grid(row=0,column=5)
        Label(self,text="  ").grid(row=0,column=6) # Dummy label for a little space before the button
        self.Mod_Set_btn=Button(self,bd=5,text="Mod",relief=RAISED,width=3,padx=5,pady=0,command=self.Modify_Settings)
        self.Mod_Set_btn.grid(row=0,column=7)

    def Modify_Settings(self):
        global AD_ResX,AdrsX1,AdrsX2,AD_Mod
        # Reload the scratchpad vars with the in use vars
        AD_ResX=AD_Res
        AdrsX1=Adrs1
        AdrsX2=Adrs2
        
        AD_Mod=Toplevel() # Turn AD_Mod into a Toplevel window
        AD_Mod.geometry("350x350")
        AD_Mod.title("AD Parameter Modification")
        Label(AD_Mod,text="Valid resolutions are 12, 14, 16, 18 which ").grid(row=0,column=0,columnspan=3,sticky=W)
        Label(AD_Mod,text="gives max SPS 240, 60, 15, 3.75 respectively.").grid(row=1,column=0,columnspan=3,sticky=W)
        Label(AD_Mod,text=" ").grid(row=2,column=0) # Blank line for spacing
        Label(AD_Mod,text="Adrs1 is the i2c address for channels 1-4").grid(row=3,column=0,columnspan=3,sticky=W)
        Label(AD_Mod,text="Adrs2 is the i2c address for channels 5-8").grid(row=4,column=0,columnspan=3,sticky=W)
        Label(AD_Mod,text="Adrs1 and Adrs2 are entered as dec NOT hex").grid(row=5,column=0,columnspan=3,sticky=W)
        Label(AD_Mod,text=" ").grid(row=6,column=0) # Blank line for spacing
        Label(AD_Mod,text="This window is a scratch pad. STRONGLY").grid(row=7,column=0,columnspan=3,sticky=W)
        Label(AD_Mod,text="SUGGEST make all changes THEN press Apply.").grid(row=8,column=0,columnspan=3,sticky=W)
        Label(AD_Mod,text="If you don't know what you are doing, press Abort NOW.").grid(row=9,column=0,columnspan=3,sticky=W)
        Label(AD_Mod,text=" ").grid(row=10,column=0) # Blank line for spacing
        
        Label(AD_Mod,text="Res:").grid(row=11,column=0)
        AD_Mod.Bit_Res_Label=Label(AD_Mod,text=str(AD_ResX),bd=5,width=3,anchor=E,relief=RIDGE,bg="yellow")
        AD_Mod.Bit_Res_Label.grid(row=11,column=1)
        AD_Mod.Bit_Res_btn=Button(AD_Mod,bd=2,text="Mod",relief=RAISED,width=3,command=self.AD_BitRes_Mod)
        AD_Mod.Bit_Res_btn.grid(row=11,column=2)
        
        Label(AD_Mod,text=" Adrs1:").grid(row=12,column=0)
        AD_Mod.BusADR1_Label=Label(AD_Mod,text=str(AdrsX1),bd=5,width=3,anchor=E,relief=RIDGE,bg="yellow")
        AD_Mod.BusADR1_Label.grid(row=12,column=1)
        AD_Mod.BusADR1_btn=Button(AD_Mod,bd=2,text="Mod",relief=RAISED,width=3,command=self.AD_Adrs1_Mod)
        AD_Mod.BusADR1_btn.grid(row=12,column=2)
        
        Label(AD_Mod,text=" Adrs2:").grid(row=13,column=0)
        AD_Mod.BusADR2_Label=Label(AD_Mod,text=str(AdrsX2),bd=5,width=3,anchor=E,relief=RIDGE,bg="yellow")
        AD_Mod.BusADR2_Label.grid(row=13,column=1)
        AD_Mod.BusADR2_btn=Button(AD_Mod,bd=2,text="Mod",relief=RAISED,width=3,command=self.AD_Adrs2_Mod)
        AD_Mod.BusADR2_btn.grid(row=13,column=2)
        
        Label(AD_Mod,text="  ").grid(row=14,column=0) # Blank line for spacing
        
        AD_Destroy_btn=Button(AD_Mod,bd=2,text="Abort",relief=RAISED,command=AD_Mod.destroy)
        AD_Destroy_btn.grid(row=15, column=0)

        AD_Apply_btn=Button(AD_Mod,bd=2,text="Apply",relief=RAISED,command=self.AD_Apply_Mod)
        AD_Apply_btn.grid(row=15, column=2)

# Modify the bit resolution value. This also detertmines rate samples per second (SPS).
    def AD_BitRes_Mod(self):
        global AD_ResX,AD_Mod
        AD_Mod.lower() # This will make this window seem to dissappear
        Results=askinteger("Integer Value","Bit Res (12,14,16,or 18)",parent=self,\
                            initialvalue=AD_ResX,minvalue=12,maxvalue=18)
        if Results != None:
            if Results == 12 or Results == 14 or Results == 16 or Results == 18:
                AD_ResX = Results
                AD_Mod.Bit_Res_Label.config(text=str(AD_ResX),width=3,anchor=E,bg="yellow")
        AD_Mod.lift() # This will make this window re-appear
        
# Modify the i2c address for channels 1-4
    def AD_Adrs1_Mod(self):
        global AdrsX1,AD_Mod
        AD_Mod.lower() # This will make this window seem to dissappear
        Results=askinteger("Integer Value","i2c Address Chans 1-4",parent=self,\
                            initialvalue=AdrsX1,minvalue=102,maxvalue=140)
        if Results != None:
            AdrsX1 = Results
            AD_Mod.BusADR1_Label.config(text=str(AdrsX1),width=3,anchor=E,bg="yellow")
        AD_Mod.lift() # This will make this window re-appear

# Modify the i2c address for channels 5-8
    def AD_Adrs2_Mod(self):
        global AdrsX2,AD_Mod
        AD_Mod.lower() # This will make this window seem to dissappear
        Results=askinteger("Integer Value","i2c Address Chans 5-8",parent=self,\
                            initialvalue=AdrsX2,minvalue=102,maxvalue=140)
        if Results != None:
            AdrsX2 = Results
            AD_Mod.BusADR2_Label.config(text=str(AdrsX2),width=3,anchor=E,bg="yellow")
        AD_Mod.lift() # This will make this window re-appear

    def AD_Apply_Mod(self):
        global AD_Mod
        global AD_ResX,AdrsX1,AdrsX2
        global AD_Res,Adrs1,Adrs2
        global AD_Bits_A,AD_Adrs1_A,AD_Adrs2_A
        AD_Res=AD_ResX
        Adrs1=AdrsX1
        Adrs2=AdrsX2
        AD_Bits_A.value=AD_Res
        AD_Adrs1_A.value=Adrs1
        AD_Adrs2_A.value=Adrs2
        AD_Set_A.value=1
        self.Bit_Res.config(text=str(AD_Res),width=3,anchor=E,bg="yellow")
        self.BusADR1.config(text=str(Adrs1),width=3,anchor=E,bg="yellow")
        self.BusADR2.config(text=str(Adrs2),width=3,anchor=E,bg="yellow")
        AD_Mod.destroy()
            
# Status message to user           
class Status_Message(LabelFrame):
    
    def __init__(self,parent,TheMessage="Start up",BgColor="black",FgColor="green",**kw):
        super(Status_Message,self).__init__(parent,relief=GROOVE,bd=5,padx=2,pady=2,
                                            fg="black",text="Status Message")
        self.message=TheMessage
        self.bgcolor=BgColor
        self.fgcolor=FgColor
        self.MsgLabel=Label(self,text=self.message,width=40,relief=GROOVE,bg=self.bgcolor,fg=self.fgcolor)
        self.MsgLabel.grid(row=0, column=0)

    def Show_Message(self,Usr_Message,Bg_Color,Fg_Color):
        self.MsgLabel.config(text=Usr_Message, bg=Bg_Color, fg=Fg_Color)

# Set up display section for Scope ON/OFF button
class Scope_On_Off(LabelFrame):
    
    def __init__(self,parent,**kw):
        
        super(Scope_On_Off,self).__init__(parent,relief=GROOVE,bd=5,padx=2,pady=2,
                                          fg="black",text="Power ON/Off",**kw)

        self.Scope_btn=Button(self,bd=5,text="POWER",relief=RAISED,width=8,padx=4,pady=2,
                              fg="black",command=self.PowerOnOff)
        self.Scope_btn.grid(row=0,column=0)


    def PowerOnOff(self):
        global ScopePower
        global MsgCode
        global adc,bus,Adrs1,Adrs2,AD_Res
        global AD_Bits_A,AD_Adrs1_A,AD_Adrs2_A,AD_Set_A
        
        if ScopePower.value==0:
            try:
                adc = ADCPi(bus, Adrs1, Adrs2, AD_Res) # Set bus, adrs1, adrs2, rate
            except IOError:
                MsgCode=5
            else:
                ScopePower.value=1
                AD_Bits_A.value=AD_Res
                AD_Adrs1_A.value=Adrs1
                AD_Adrs2_A.value=Adrs2
                AD_Set_A.value=1
                self.Scope_btn.config(fg="green")
        else:
            ScopePower.value=0
            self.Scope_btn.config(fg="black")

        
# This is the main app frame. The GUI is driven from here and
# almost all of the non-GUI code is in here.
class App(Frame):
    def __init__(self,parent=None,relief=RAISED,bd=5,padx=5,pady=5, **kw):
        super(App,self).__init__(parent,**kw)
        self.grid() # Link this frame to grid geometry manager
        self.parent = parent
        # Use real BCM pin numbering rather than P1 pinout numbering.
        # Not sure what the future will bring and BCM covers them all.
        GPIO.setmode(GPIO.BCM)
        # Initialize the status message clear timer vars
        self.StatusMsg_Clear_Tmr_Run=False
        self.StatusMsg_Clear_Tmr_Complete=time.time()
                
        ####################################################################
        ###### THIS SECTION LAYS OUT THE MAIN GRAPHICS WINDOW ##############
        ####################################################################
        
        # Place scope trace area on screen
        self.TraceGraph=Trace(self)
        self.TraceGraph.grid(row=1,column=1,rowspan=4,columnspan=2)
        # Place A/D channel select on screen
        self.AD_SelGraphA=AD_Channel_Sel(self)
        self.AD_SelGraphA.grid(row=0,column=1)
        # Place A/D channel settings on screen
        self.AD_SetGraphA=AD_Settings(self)
        self.AD_SetGraphA.grid(row=0,column=2)
        # Place Threshold select on screen
        self.ThreshAGraph=Threshold_Set_A(self)
        self.ThreshAGraph.grid(row=1,column=0,sticky=N)
        # Place Sweep select on screen
        self.SweepAGraph=Sweep_Select_A(self)
        self.SweepAGraph.grid(row=1,column=0,sticky=S)
        # Place power on/off button on screen
        self.PowOnOff=Scope_On_Off(self)
        self.PowOnOff.grid(row=5,column=0)
        # Place X axis scale time select on screen
        self.XScaleGraph=XScale_Select(self)
        self.XScaleGraph.grid(row=5,column=1)
        # Place the trigger select on screen
        self.TrigGraph=Trigger_Sel_A(self)
        self.TrigGraph.grid(row=1,column=3,rowspan=2)
        # Place user status messages section
        self.StatusMessage=Status_Message(self,"Auto","black","yellow")
        self.StatusMessage.grid(row=5,column=2)
        
        ###################################################################
        ############## END OF MAIN GRAPHIC WINDOW LAYOUT ##################
        ###################################################################
        
        self.Update_All()# Start scan loop here inside the main class app by invoking update method

    # Runs on a set ms cycle to check the state of the A/D and GPIO inputs.
    # SEE THE LAST LINE IN THIS METHOD FOR THE CYCLE TIME.                  
    def Update_All(self):

        global x1,y1,x2,y2
        global LinePtr, Max_Lines
        global Y_Lab_width,X_Lab_height
        global Xscale,Yscale
        global Y_Axis_Pixels,X_Axis_Pixels
        global Volts_y1,Volts_y2
        global FaultCode
        global Time_x1,Time_x2,TraceStartTime
        global StartNewTrace
        global LastLine,ContinuousSweepA,SingleSweepA
        global Trigger_Sel,Trigger_Select,TrigThresh_A
        global NewXscale
        global MsgCode,OldMsgCode
        global Trig_Pin_Conf,Trig_Pin_GPIO
        global ScopePower, AD_Error_A

    # User selected a new X scale on the fly.
        if NewXscale > 0:
            if NewXscale == 1:
                self.TraceGraph.Half_Second_X()
            if NewXscale == 2:
                self.TraceGraph.One_Second_X()
            if NewXscale == 3:
                self.TraceGraph.Two_Second_X()
            if NewXscale == 4:
                self.TraceGraph.Three_Second_X()
            if NewXscale == 5:
                self.TraceGraph.Four_Second_X()
            NewXscale=0
            StartNewTrace=True
                            
        # Process a new status message. This code needs to be ahead of the code below.
        if AD_Error_A.value != 0:
            MsgCode = 5
            AD_Error_A.value=0
            if ScopePower.value==1:
                self.PowOnOff.PowerOnOff()
        if ScopePower.value==0 and MsgCode!=5:
            MsgCode=4
        if MsgCode != OldMsgCode:
            if MsgCode == 0:
                self.StatusMessage.Show_Message("Running","black","green")
            if MsgCode == 1:
                self.StatusMessage.Show_Message("A/D Task Not Running. Check i2c address.","red","yellow")
            if MsgCode == 2:
                self.StatusMessage.Show_Message("Waiting For Trigger","black","green")
            if MsgCode == 3:
                self.StatusMessage.Show_Message("Please Configure Before Selecting","yellow","red")
                self.StatusMsg_Clear_Tmr_Run=True
                self.StatusMsg_Clear_Tmr_Complete=time.time()+3.0
            if MsgCode == 4:
                self.StatusMessage.Show_Message("Power is Off. Is i2c adrs ok before turning on?","black","green")
            if MsgCode == 5:
                self.StatusMessage.Show_Message("Problem with i2c address.","yellow","red")
                self.StatusMsg_Clear_Tmr_Run=True
                self.StatusMsg_Clear_Tmr_Complete=time.time()+3.0
            OldMsgCode=MsgCode

    # If trigger mode is activated, wait for the last sweep to finish, then check for
    # an appropriate trigger before drawing a new trace. For now, traces will continue
    # in single sweep mode as long as the selected trigger is active
        if Trigger_Sel and not SingleSweepA:
            nn = Trigger_Select.get()
            if (nn == 9) and (ADvalue_A.value >= TrigThresh_A):
                SingleSweepA=True
            if (nn > 0 and nn < 9):
                if Trig_Pin_Conf[nn] and GPIO.input(Trig_Pin_GPIO[nn]):
                    SingleSweepA=True

# Draw a trace if power is on and either sweep selection is true.
        if ScopePower.value==1 and (ContinuousSweepA or SingleSweepA):
            if StartNewTrace: # Initialize for the first trace line
                self.TraceGraph.New_Trace()
                StartNewTrace=False
                if FaultCode == 0 and not self.StatusMsg_Clear_Tmr_Run:
                    MsgCode = 0

    # The end point of the previous line is the begin point of the next line, so move
    # the previous line's x2,y2 to the new x1,y1 before calculating the new x2,y2
    # Also, move the volts in case threshold check is on.
            x1=x2
            y1=y2
            Volts_y1=Volts_y2
        
            # Get a new x2,y2
 #           Time_x2=ReadTime_A.value
            TryNumber=0
            SecondPointTest=False
            # Spin around here waiting for the AD converter routine to give us a new reading. We know
            # this when the ReadTime value increases after the AD read completes.
            while not SecondPointTest:
                if Time_x2 == ReadTime_A.value:
                    time.sleep(0.001) # Wait a millisecond and go read again
                    TryNumber += 1
                    if TryNumber > 1000: # 1000 tries is 1 second. AD should be faster than that
                        FaultCode=3 # AD program isn't running or updating time for some reason
                        SecondPointTest=True
                        break
                else:
                    Time_x2=ReadTime_A.value
                    Volts_y2=ADvalue_A.value
                    SecondPointTest=True

            if FaultCode == 0:
            # The begin point (x1,y1) is already in pixels, now convert the end point to pixel coords
            # including the offset of the location of the y axis.
                x2=round((Time_x2-TraceStartTime)* Xscale)+Y_Lab_width
                if x2 >= pix_width:
                    x2 = pix_width
                    LastLine=True
                y2=round(Volts_y2 * Yscale)
                if y2 < 1:
                    y2 = 1 # Keep over voltage on the screen (top of Y axis)
                if y2 > Y_Axis_Pixels:
                    y2 = Y_Axis_Pixels # Keep under voltage on the screen (bottom of Y axis)
                y2=Y_Axis_Pixels-y2 #Invert y-axis location (0 is top pixel on a canvas)

                LinePtr += 1 #Point to next line in huge line array
                # Don't let the pointer outside the array size. Bump this if the array is
                # made larger above due to a faster Pi or A/D board.
                if LinePtr >= (Max_Lines-2):
                    LastLine=True
                Color_Me = "green"
                if Thresh_A and ((Volts_y1 >= Threshold_A) or (Volts_y2 >= Threshold_A)):
                    Color_Me= "red"
                self.TraceGraph.Draw_Line(Color_Me)
            else:
                #print("Fault Code failure: ",FaultCode)
                MsgCode=1
                #root.quit()
        else:
            if FaultCode == 0 and not self.StatusMsg_Clear_Tmr_Run:
                MsgCode = 2


        # Status message clear timer
        if self.StatusMsg_Clear_Tmr_Run and (time.time() >= self.StatusMsg_Clear_Tmr_Complete):
            self.StatusMsg_Clear_Tmr_Run = False
            MsgCode=0
            
# Continuously run "Update_All", but take a 1 millisecond breath between runs.
        self._timer = self.after(1,self.Update_All)

        
    # This is used to run the Rpi.GPIO cleanup() method to return pins to be an input
    # and then destroy the app and its parent.
    def onClose(self):
# !!!!!!!!!!!!!!!!MAKE SURE TO UNCOMMENT GPIO.cleanup WHEN GPIO IS USED IN THE CODE!!!!!!!!!!!!!!!!!!
#        GPIO.cleanup()
        self.destroy()
        self.parent.destroy()
        
########################### RUN IT ##############################
if __name__ == '__main__':

    # Start up the concurrent A/D reader task
    p1 = Process(target=ADC_Reader_A,name='ADC_Reader_A',args=(ADchannel_A,ADvalue_A,ReadTime_A,AD_Bits_A,
                                                               AD_Adrs1_A,AD_Adrs2_A,AD_Set_A,
                                                               ScopePower,AD_Error_A))
    p1.daemon=True # Setting the daemon True should prevent orphan process when parent exits
    p1.start()

    # Set up and display main GUI window
    root = Tk()
    Sel_Chan = IntVar() 
    ThreshA_On_Off = IntVar()
    SweepSelectA= IntVar()
    XScaleSelect= IntVar()
    Trigger_Select= IntVar()
    Config_Select= IntVar()
    Dir_Select=IntVar()
    Res_Select=IntVar()
    
    root.title("Simple Oscilloscope ABE")
    OScope = App(root)
    # When the window is closed, run the onClose function.
    root.protocol("WM_DELETE_WINDOW",OScope.onClose)
    root.mainloop()

########################### Done ###################################
