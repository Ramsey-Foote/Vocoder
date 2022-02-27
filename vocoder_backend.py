# -*- coding: utf-8 -*-
"""
Created on Sat Feb 26 19:16:18 2022

@author: ramsey
"""

import numpy as np
import scipy as spi
from scipy import signal
import matplotlib.pyplot as plt
import pyaudio
import struct
from scipy.io import wavfile
import IPython


#General Variables
fs = 44100
carrier_frequency=1000
num_filters=12
smooth_hz=1/.01
cutoff = smooth_hz/(44100/2.)

#Filtering Methods
def butter_bandpass(l, h, fs, order=5):
    nyquist=.5*fs
    low = l/nyquist
    high=h/nyquist
    b,a=signal.butter(order,[low,high], btype='band')
    return b,a
def butter_bandpass_filter(data,l,h,fs,order=5):
    b,a=butter_bandpass(l,h,fs,order=order)
    y=signal.lfilter(b,a,data)
    return y

#get audio
samrate, dat = wavfile.read('test.wav')
dat=dat[:,0]#get one chanel of audio
dat=dat[int(len(dat)/4):int(len(dat)/2)]#cut length to save time, delete later

#generate bins
bins=np.zeros(num_filters)
bins[0]=100
for i in range(num_filters-1):
    bins[i+1]=(i+1)*1000
    
#generate and filter carrier
t=np.linspace(0,len(dat)/fs,len(dat))
carrier = signal.square(2*np.pi*carrier_frequency*t,duty=.5) 
FC = np.zeros((num_filters,len(carrier)))
for i in range(num_filters-1):
    FC[i]=butter_bandpass_filter(carrier,bins[i],bins[i+1],fs)
    
#Filter audio
FS = np.zeros((num_filters,len(dat)))
for i in range(num_filters-1):
    FS[i]=butter_bandpass_filter(dat,bins[i],bins[i+1],fs)
    
#apply envelope to audio bands
b,a = signal.butter(1,cutoff,btype='low',analog=False)
env=np.zeros((num_filters,len(FS[0])))
for i in range(num_filters):
    env[i]=np.sqrt(signal.lfilter(b,a,FS[0]**2))
env[np.isnan(env)]=0

#Multiply carrier and audio bands
To_add=np.zeros((num_filters,len(env[0])))
for i in range(num_filters):
    To_add[i]=FS[i]*FC[i]
    
#Add outputs together
out=np.zeros(len(env[0]))
for i in range(len(env[0])):
    out[i]=sum(To_add[:,i])

#plot audio
plt.figure()
plt.plot(out)

#play audio
IPython.display.Audio(out,rate=samrate)
print(samrate)