#!/usr/bin/env python
import sys
import os
import rospy
import rosgraph
import time
from roslib.message import get_message_class
from std_msgs.msg import *
from edg_ur10.srv import *
from rospy.msg import AnyMsg
import numbers
import collections
from operator import attrgetter

from datetime import datetime


#Current state of logging
isLoggingEnabled = False
logJustStarted = 0
#List of topics to record
listOfTopics = []

######################## Tae
#List of Attributes per each Topic (i.e. "topic 1 : [attr1, attr2, attr3]")
listOfAttributesInTopic = {}
#call will check if the dictionary above is filled for the given topic

#Data record before it gets dumped in the CSV file
record = {}
# record should be topic/listOfAttribute : Value, rather than topic:Value



######################################33

#List of Subscribers so we can easily Unsubscribe to topics.
listOfSubscribers = []

#List of available topics and their corresponding types
topic_types = []


#Output CSV file used, a new file is generated every time the user starts a
#new data logging process.
output_file_name = {}
output_file = {}
all_output_file_names = ""

#New appendData, it makes files for each topic
def appendDataPoint(topic, msg):
    global record
    global listOfAttributesInTopic
    global listOfTopics
    global output_file
    global logJustStarted
    
    #Take values of attributes in the msg.
    thisDataDict = {}
    for attributName in list(listOfAttributesInTopic[topic].keys()):
        retriever = attrgetter(attributName)
        value = retriever(msg)
        thisDataDict.update({attributName:value})
    
    #check if header has stamp    
    thisTimeStamp = rospy.get_rostime()        
    
    # retrive header time stamp if exist
    if hasattr(msg, "header"):
        try:
            retriever = attrgetter("header.stamp")
            timeVal = retriever(msg)
            if timeVal.secs > 0:
                thisTimeStamp = timeVal
        except Exception:
            pass        
    
    line = str(thisTimeStamp)+','

    #For each topic, append its value    
    for attributName in list(listOfAttributesInTopic[topic].keys()):
        try:
            thisDataStr = str(thisDataDict[attributName])
            if listOfAttributesInTopic[topic][attributName] >1: # value is list or tuple
                thisDataStr = thisDataStr[1:-1]
            line += thisDataStr + ','
        except KeyError as e:
            line += ','
    line = line[0:-1] + '\n'        
    output_file[topic].write(line)



#Wait until we get at least one value from each topic and then
#append the CSV file with the informations.

def updateListofAttribute(topic, msg, attributeName=""):
    global listOfAttributesInTopic
    #Search for the attribute that is numeric
    # print attributeName
    
    if hasattr(msg, "__slots__"):        
        thisSlotList = msg.__slots__ 
        for tempSlotName in thisSlotList:
            if tempSlotName != "header": 
                updateListofAttribute(topic,getattr(msg, tempSlotName), attributeName+"."+tempSlotName)
    
    elif isinstance(msg, numbers.Number): # check if the attributed msg is single Number
        if topic not in listOfAttributesInTopic:
            listOfAttributesInTopic[topic]={}
        
        listOfAttributesInTopic[topic].update({attributeName[1:]:1}) # remove the front dot in the attribute name
        return

    elif (isinstance(msg, list) or isinstance(msg, tuple)) and (bool(msg) and isinstance(msg[0], numbers.Number)) : # check if the attributed msg is List
        if topic not in listOfAttributesInTopic:
            listOfAttributesInTopic[topic]={}
        
        listOfAttributesInTopic[topic].update({attributeName[1:]:len(msg)}) # remove the front dot in the attribute name
        return        
    return

def writeFileHeader(topic):
    global listOfAttributesInTopic
    global output_file
    #Write the header of the CSV file with all the topic names
    header = 'ROStimestamp,'    
    for attributName in list(listOfAttributesInTopic[topic].keys()):
        for i in range(listOfAttributesInTopic[topic][attributName]):
            header = header + topic +'.' + attributName + ','

    header = header[0:-1] + '\n'
    output_file[topic].write(header)
    


#This callback function is triggered every time a message was sent to a
#subscribed topic.
def callback(data, args):
    global listOfAttributesInTopic


    #Assume that the first argument represents the name of the type of the message
    topic = args[0]
    msg_name = args[1]
    # print(args)

    #Retrieve the class associated with that message name
    msg_class = get_message_class(msg_name)
    #Transform the message data into an instance of the message class
    msg = msg_class().deserialize(data._buff)

    # add the topic to the attribute list
    if topic not in listOfAttributesInTopic:
        #update the list of attributes in that topic
        updateListofAttribute(topic, msg)
        
        #sort List of attributes and write headers for each file
        listOfAttributesInTopic[topic] = collections.OrderedDict(sorted(listOfAttributesInTopic[topic].items()))
        writeFileHeader(topic)
    
    appendDataPoint(topic, msg)
    



#Iterates over the list of subscribers and unregister them in order to stop
#the callback functions from being triggered.
def unsubscribeAllTopics():
    global listOfTopics
    global listOfSubscribers
    global listOfAttributesInTopic

    for sub in listOfSubscribers:
        sub.unregister()

    listOfTopics = list()
    listOfSubscribers = list()
    listOfAttributesInTopic = {}

#Reads a list of ROS topics from the configuration file into a global list
def loadConfigFile(filePath):
    global listOfTopics
    global listOfSubscribers
    global topic_types
    #Reset the subscription
    unsubscribeAllTopics()

    #Iterates over every line of the configuration file and subscribe to each
    #topic.
    with open(filePath) as fp:
        for line in fp:
            #The line contains one or more newline characters that needs to be removed
            topic = line.replace('\n','').replace('\r','').replace(' ','')
            #Retrieve the message type associated with that topic
            msg_name = ""
            for couple in topic_types:
                tp = couple[0]
                ty = couple[1]
                if tp == topic:
                    msg_name = ty
            print("Input Result")                
            print(topic)
            print(msg_name)
            #As not every topic uses the same type of message, AnyMsg is used.
            #http://docs.ros.org/melodic/api/rospy/html/rospy.msg.AnyMsg-class.html
            #The message name is passed as an argument so the callback function can use it
            sub = rospy.Subscriber(topic, AnyMsg, callback, (topic,msg_name))
            #It is very important that the n-th subscriber correponds to the n-th topic
            #in these lists as its assumed afterward.
            listOfTopics.append(topic)
            listOfSubscribers.append(sub)
            print("Msg Name")
            print(msg_name)


#This callback function sets the state of the logging process (Enabled or Disable)
#when the user calls this service.
def setLoggingState(request):
    global isLoggingEnabled
    global output_file
    global output_file_name
    global all_output_file_names
    global listOfTopics
    global record
    global logJustStarted

    #This field of the request object contains the value set by the user when
    #calling this service.
    desiredState = request.EnableDataLogging

    #Enable logging
    if desiredState == True:
        #If its not already in that state
        if isLoggingEnabled != desiredState:
            #Loads the tae_datalogger/config/TopicsList.txt file
            loadConfigFile(os.path.expanduser('~') + '/catkin_ws/src/edg_ur10/config/TopicsList.txt')
            print(("Listening for these topics: "+str(listOfTopics)))

            currentTimeStr = datetime.now().strftime('%Y_%m%d_%H%M%S')

            all_output_file_names = ""
            for topicName in listOfTopics:
                #Generate a new for the output CSV file for each topic
                output_file_name[topicName] = '/tmp/dataLog_' +currentTimeStr+ '_' + topicName[1:] + '.csv'    
                # output_file_name = '/tmp/data_log_'+str(int(time.time()))+'.csv'
                #Open the CSV file for writing
                output_file[topicName] = open(output_file_name[topicName],'w')
                print(("Writing output to: "+output_file_name[topicName]))
                all_output_file_names += output_file_name[topicName] + " "
            logJustStarted = 1
            
            
        print("Data logging is started.")

    #Disable logging
    if desiredState == False:
        #If its not already in that state
        if isLoggingEnabled != desiredState:
            for topicName in listOfTopics:
                output_file[topicName].close()

            unsubscribeAllTopics()
            all_output_file_names = ""
            record = {}
            logJustStarted = 0
        print("Data logging is stopped.")

    isLoggingEnabled = desiredState

    #This is what gets returned to the user
    return EnableResponse(all_output_file_names)


if __name__ == '__main__':

    # For Debugging
    import subprocess
    # subprocess.Popen("roscore")
    
    print("DataLogger Started")
    

    #Retrieve the graph of nodes
    master = rosgraph.Master(rospy.get_name())

    #Get a list of all topics and their associated message type
    topic_types = master.getTopicTypes()
    
    rospy.init_node("edg_log_node", anonymous=True)
    
    
    #Advertise our service
    service = rospy.Service('data_logging', Enable, setLoggingState)
    
    # For Debugging    
    # subprocess.call(["rosservice", "call", "/data_logging", "1"]) #enable
    
    # subprocess.call(["rosrun", "netft_utils", "netft_utils_sim"])
    
    
    rospy.spin()
    
