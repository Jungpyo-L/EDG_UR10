import os, sys
import pandas as pd
import re
import scipy.io 

if not os.path.exists('/tmp/processed_csvs'):
    os.makedirs('/tmp/processed_csvs')

# First Check all CSV files in /tmp/ and bring them in as a variable  
fileList = []
for file in os.listdir("/tmp"):
    if file.endswith(".csv"):
        fileList.append(os.path.join("/tmp", file))

savingDictionary = {}
for fileName in fileList:
    print(fileName)
    try:
        df=pd.read_csv(fileName)             
                             
        thisColumnName = df.columns.tolist()
        
        splitedList = re.split('_|\.', fileName)        
        thisTopicName = ''.join(splitedList[4:-1])        
        
        savingDictionary[thisTopicName+"_columnName"] = thisColumnName
        savingDictionary[thisTopicName+"_data"] = df.values
    except Exception as e:
        print(str(e))
        pass
    #move to temparary folder    
    os.rename(fileName, '/tmp/processed_csvs/' + re.split('/',fileName)[-1])
    


savingFileName = os.path.expanduser('~') + '/DataLog_'+ '_'.join(splitedList[1:4])   
scipy.io.savemat(savingFileName, savingDictionary)