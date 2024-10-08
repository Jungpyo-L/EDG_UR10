#!/usr/bin/env python
from ast import arg
import os
from datetime import datetime
import numpy as np
import pandas as pd
from scipy.io import savemat
import re

class fileSaveHelp(object):
    def __init__(self, savingFolderName = 'EDG_Experiment'):
        self.savingFolderName = savingFolderName
        self.ResultSavingDirectory = os.path.expanduser('~') + '/' + self.savingFolderName + '/' + datetime.now().strftime("%y%m%d")        
        if not os.path.exists(self.ResultSavingDirectory):
            os.makedirs(self.ResultSavingDirectory)
        
    def getLastMatFileSaved(self):    
        fileList = []
        for file in os.listdir(self.ResultSavingDirectory):
            if file.endswith(".mat"):
                fileList.append(file)
        try:
            return sorted(fileList)[-1]
        except Exception as e:
            print(e)
        return "none"
    
    # Clear all csv files in the tmp folder
    def clearTmpFolder(self):
        fileList = []
        for file in os.listdir("/tmp"):
            if file.endswith(".csv"):
                fileList.append(os.path.join("/tmp", file))        
        for fileName in fileList:
            os.remove(fileName)


    def saveDataParams(self, args=None, appendTxt = ''):    

        #check if CSV Files are available
        tmp_dummyFolder = '/tmp/processed_csv'
        if not os.path.exists(tmp_dummyFolder):
            os.makedirs(tmp_dummyFolder)
        
        # First Check all CSV files in /tmp/ and bring them in as a variable  
        fileList = []
        for file in os.listdir("/tmp"):
            if file.endswith(".csv"):
                fileList.append(os.path.join("/tmp", file))
                
        print("csv files: ", fileList)
        print("grabbing columns from csv files into one dataframe")
        savingDictionary = {}
        errorCount = 0
        for fileName in fileList:
            print("trying file: ", fileName)
            try:    
                df=pd.read_csv(fileName)             
                                    
                thisColumnName = df.columns.tolist()
                
                splitedList = re.split('_|\.', fileName)        
                thisTopicName = ''.join(splitedList[4:-1])        
                
                savingDictionary[thisTopicName+"_columnName"] = thisColumnName
                savingDictionary[thisTopicName+"_data"] = df.values
                #move to temparary folder    
                os.rename(fileName, tmp_dummyFolder + '/' + re.split('/',fileName)[-1])
            except Exception as e:
                print(e)
                errorCount +=1

        if errorCount > 0:
            print("!!!!-- Mised ", errorCount, " csv files --!!!!")

        # Save all the contents in the args as variables
        if args is not None:
            argsDic = vars(args)
            for key in list(argsDic.keys()):
                savingDictionary[key]=argsDic[key]
        

        savingFileName_noDir = 'DataLog_'+ '_'.join(splitedList[1:4])
        savingFileName = self.ResultSavingDirectory + '/' + savingFileName_noDir + '_' + appendTxt + '.mat'
        print(savingFileName)

        savemat(savingFileName, savingDictionary)
        print("savingFileName_noDir: ", savingFileName_noDir)

        return self.ResultSavingDirectory, savingFileName_noDir


