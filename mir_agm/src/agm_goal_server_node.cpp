//Author: Michael Muldoon
//email: michael.muldoon.home@gmail.com
//license: Apache 2.0
//Comment: This node is a service server connecting to webserver to get the next
// manufacturing step for the MIR delivery robot to fullfill

#include <ros/ros.h>
#include <mir_agm/NextGoal.h>
#include <curl/curl.h>
#include <string>

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

using namespace std;

bool get_next_order(mir_agm::NextGoal::Request &req,
                    mir_agm::NextGoal::Response &res)
{
    CURL *curl;
    CURLcode res_curl;
    string readBuffer;
    string delimiter = ",";
        
    curl = curl_easy_init();
    
    if(curl) {
        string url = "http://192.168.2.238:3001/workerGetNextJob?name=" + req.name;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res_curl = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        cout<<readBuffer<<endl;
        //res.name = readBuffer;
        //res.body = readBuffer.substr(0, readBuffer.find(delimiter));
        //res.drink = readBuffer.substr(readBuffer.find(delimiter)+1, readBuffer.length());
    }
    
    //cout<<"Response: "<<res<<endl;
    return true;
}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "next_order_server");
    ros::NodeHandle n;

    ros::ServiceServer server = n.advertiseService("/next_order", 
        get_next_order);
  
    ros::spin();

}

