#!/usr/bin/env python

import numpy as np
from scipy.signal import butter, lfilter, freqz, filtfilt
from scipy.stats import skew
from scipy.stats import kurtosis
import matplotlib.pyplot as plt
from scipy.fftpack  import fft, fftfreq
import pandas as pd

import seaborn as sns
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets

import seaborn as sns
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets
from sklearn.metrics import accuracy_score, precision_score, confusion_matrix
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.model_selection import cross_val_score,StratifiedKFold
from sklearn.decomposition import PCA

def butter_lowpass(cutoff, fs, order=5):
    return butter(order, cutoff, fs=fs, btype='low', analog=False)

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_lowpass__IIR_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    # print(a, b)
    b = np.array([0.0825404703305007,	0.0825404703305007,	0])
    a = np.array([1,	-0.834919059338999,	0])
    y = filtfilt(b, a, data)
    return y

def filterIIR(data):
    order = 1
    footLenght = 0.13891
    fs = 400.0       # sample rate, Hz
    cutoff = 1/footLenght  # desired cutoff frequency of the filter, Hz
    # Filter the data, and plot both the original and filtered signals.
    return butter_lowpass__IIR_filter(data, cutoff, fs, order)

def load_profile_from_csv(path, col=1):
    df = pd.read_csv(path, header=None)
    data = df[col]
    return data

def load_profile_from_ros():
    pass

def plotFilter(data,data_filtered):
    plt.plot(data, 'b-', label='data')
    plt.plot(data_filtered, 'g-', linewidth=2, label='filtered data')
    plt.xlabel('Time [sec]')
    plt.grid()
    plt.legend()
    plt.subplots_adjust(hspace=0.35)
    plt.show()

def parametersCalculation(data):
    Z_new = data-np.nanmean(data)
    Sa = np.mean(np.absolute(Z_new))
    Sq = np.std(Z_new)
    Ssk = skew(Z_new)
    Sku = kurtosis(Z_new, fisher=False) # Pearson Kurtosis
    Sv = np.absolute(min(Z_new))
    Sp = np.absolute(max(Z_new))
    Sz = Sv + Sp

    M = np.var(Z_new)
    f_controller = 200
    dt = 1.0/f_controller
    dx = Z_new[1:] - Z_new[0:-1]
    P = np.var(dx/dt)
    z_fft = fft(Z_new)
    nomi = [0 for i in range(len(z_fft))]
    deni = [0 for i in range(len(z_fft))]
    for i in range(0, len(z_fft)):
        nomi[i] = np.absolute(z_fft[i])**4-np.absolute(z_fft[0])**4
        deni[i] = np.absolute(z_fft[i])**2-np.absolute(z_fft[0])**2
    sumNomi = 0
    sumDeni = 0
    for i in range(0, len(nomi)):    
        sumNomi = sumNomi + nomi[i]; 
        sumDeni = sumDeni + deni[i];    
    K = 1 - (sumNomi/sumDeni**2)
    return Sa,Sq,Ssk,Sku,Sv,Sp,M,P,K    

def load_csv(path):
    df = pd.read_csv(path,header=None)
    data = df.to_numpy()
    return data

def scale_data(train_data,data):
    scaler = StandardScaler()
    scaler.fit(train_data)
    scaled_data = scaler.transform(data)
    return scaled_data



if __name__ == '__main__':

    # Load the ground profile data
    file = '/home/jrluser/my_workspace/controllers/lipm_walking_controller/scripts/profile.csv'
    profile = load_profile_from_csv(file,0)

    # Filter profile 
    profile_filtered = filterIIR(profile)

    # Plot profile and profile filtered 
    # plotFilter(profile,profile_filtered)

    # Calculate roughness parameters
    Sa,Sq,Ssk,Sku,Sv,Sp,M,P,K = parametersCalculation(profile_filtered)
    roughness = [Sp,P,P**2,Sku**3,P**3,Sp**3]
    # print(Sa,Sq,Ssk,Sku,Sv,Sp,M,P,K)

    # KNN classification
    # Import data from Matlab for model evaluation
    filetrainData = '/home/jrluser/my_workspace/controllers/lipm_walking_controller/scripts/trainData.csv'
    filetestData = '/home/jrluser/my_workspace/controllers/lipm_walking_controller/scripts/testData.csv'
    trainData = load_csv(filetrainData)
    XtrainData = trainData[:,0:6]
    ytrainData = trainData[:,6]
    testData = load_csv(filetestData)
    XtestData = testData[:,0:6]
    ytestData = testData[:,6]



    # Scale data
    XtrainData = scale_data(XtrainData,XtrainData)
    XtestData = scale_data(XtrainData,XtestData)
    # Create an instance of Neighbours Classifier and fit the data.
    n_neighbors = 10
    clf = neighbors.KNeighborsClassifier(n_neighbors, weights="distance", metric="euclidean", algorithm="kd_tree")
    clf.fit(XtrainData, ytrainData)

    stiffness_group = clf.predict([roughness])
    # stiffness_group = clf.predict(XtestData)
    print(stiffness_group)
    