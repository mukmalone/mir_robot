//Author: Michael Muldoon
//email: michael.muldoon.home@gmail.com
//license: Apache 2.0
//Comment: This node is a service server connecting to webserver to get the next
// manufacturing step for the MIR delivery robot to fullfill

#include <ros/ros.h>
#include <mir_agm/WebComm.h>
#include <curl/curl.h>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>

using namespace std;

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

void decomposeRes(string const &str, const char delim,
                vector<std::string> &out) {
    // construct a stream from the string
    stringstream ss(str);

    string s;
    while (getline(ss, s, delim)){
        out.push_back(s);
    }
}

bool get_next_order(mir_agm::WebComm::Request &req,
                    mir_agm::WebComm::Response &res)
{
    CURL *curl;
    CURLcode res_curl;
    string readBuffer;
    const char delimiter = ',';
        
    curl = curl_easy_init();
    
    if(curl) {            
        string url = "http://192.168.2.238:3001"; //home
        //string url = "http://10.100.3.167:3001"; //office
        string f = req.function;
        if(f=="NEXTJOB"){
            url += "/workerGetNextJob";
        } else if (f=="ACTIVATEJOB") {
            url += "/workerActivateJob";
        } else if (f=="MOVEWORKER") {
            url += "/workerLocation";
        } else if (f=="TAKEPART") {
            url += "/workerTakePart";
        } else if (f=="LOADPART") {
            url += "/workerLoadPart";
        } else if (f=="ARCHIVEJOB") {
            url += "/workerArchiveJob";
        } else {
            //do nothing
        }            

        url += "?key="+req.key+"&location="+req.location;
        
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res_curl = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        //remove { } from incoming message
        readBuffer.erase(remove(readBuffer.begin(), readBuffer.end(), '{'), readBuffer.end());
        readBuffer.erase(remove(readBuffer.begin(), readBuffer.end(), '}'), readBuffer.end());
        readBuffer.erase(remove(readBuffer.begin(), readBuffer.end(), '"'), readBuffer.end());        
        cout<<readBuffer<<endl;

        //break string into a vector
        vector<string> out;
        decomposeRes(readBuffer, delimiter, out);
        
        //build response
        int cnt=0;
        for(auto &s: out){
            string temp = s.substr(s.find(':')+1, s.length());
            cout<<cnt<<":"<<temp<<endl;
            switch(cnt) {
                case 0:
                    //status
                    res.status=stoi(temp);                    
                    break;
                case 1:
                    //name
                    res.name=temp;
                    break; 
                case 2:   
                    //tools             
                    res.tools=temp;
                    break; 
                case 3:
                    //program
                    res.program=temp;
                    break;
                case 4:
                    //poNum
                    res.poNum=temp;
                    break;
                case 5:
                    //partNumber
                    res.partNumber=temp;
                    break;
                case 6:
                    //partSerialNumber
                    res.partSerialNumber=temp;
                    break;
                case 7:
                    //sourceName
                    res.sourceName=temp;
                    break;
                case 8:
                    //sourcePosX
                    res.sourcePosX=stoi(temp);
                    break;
                case 9:
                    //sourcePosY
                    res.sourcePosY=stoi(temp);
                    break;
                case 10:
                    //sourcePosZ
                    res.sourcePosZ=stoi(temp);
                    break;
                case 11:
                    //sourceOrientX
                    res.sourceOrientX=stoi(temp);
                    break;
                case 12:   
                    //sourceOrientY
                    res.sourceOrientY=stoi(temp);        
                    break;
                case 13:   
                    //sourceOrientZ
                    res.sourceOrientZ=stoi(temp);        
                    break;    
                case 14:   
                    //sourceOrientW
                    res.sourceOrientW=stoi(temp);        
                    break;
                case 15:
                    //destName
                    res.destName=temp;
                    break;
                case 16:
                    //destPosX
                    res.destPosX=stoi(temp);
                    break;
                case 17:
                    //destPosY
                    res.destPosY=stoi(temp);
                    break;
                case 18:
                    //destPosZ
                    res.destPosZ=stoi(temp);
                    break;
                case 19:
                    //destOrientX
                    res.destOrientX=stoi(temp);
                    break;
                case 20:   
                    //destOrientY
                    res.destOrientY=stoi(temp);        
                    break;
                case 21:   
                    //destOrientZ
                    res.destOrientZ=stoi(temp);        
                    break;    
                case 22:   
                    //destOrientW
                    res.destOrientW=stoi(temp);        
                    break;
                default:
                    break;
                    //cout <<s<<'\n';
            }
            cnt++;
        }
    }
    
    //cout<<"Response: "<<res<<endl;
    return true;
}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "web_comm_server");
    ros::NodeHandle n;

    ros::ServiceServer server = n.advertiseService("/web_comm", 
        get_next_order);
  
    ros::spin();

}

