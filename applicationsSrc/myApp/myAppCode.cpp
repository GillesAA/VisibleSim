/**
 * @file myAppCode.cpp
 * Generated by VisibleSim BlockCode Generator
 * https://services-stgi.pu-pm.univ-fcomte.fr/visiblesim/generator.php#
 * @author yourName
 * @date 2022-09-22
 **/

#include "myAppCode.hpp"

MyAppCode::MyAppCode(BlinkyBlocksBlock *host):BlinkyBlocksBlockCode(host),module(host) {
          // @warning Do not remove block below, as a blockcode with a NULL host might be created
    //  for command line parsing
    if (not host) return;

    // Registers a callback (myBroadcastFunc) to the message of type R
    addMessageEventFunc2(BROADCAST_MSG_ID,
                         std::bind(&MyAppCode::myBroadcastFunc,this,
                                   std::placeholders::_1, std::placeholders::_2));

    // Registers a callback (myAcknowledgeFunc) to the message of type K
    addMessageEventFunc2(ACKNOWLEDGE_MSG_ID,
                         std::bind(&MyAppCode::myAcknowledgeFunc,this,
                                   std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(TOMAX_MSG_ID,
                         std::bind(&MyAppCode::toMaxFunc,this,
                                   std::placeholders::_1, std::placeholders::_2));

          }

    void MyAppCode::startup() {
        console << "start " << getId() << "\n";

        if (isA) {
            module->setColor(RED);
            currentRound=1;
            distance=0;
            maxd=distance;
            nbWaitedAnswers=sendMessageToAllNeighbors("distance(1,1)",new MessageOf<tuple<int,int,int>>(BROADCAST_MSG_ID,make_tuple(distance+1,currentRound,maxd)),1000,100,0);
        } else {
          currentRound=0;
        }
    }

    void MyAppCode::myBroadcastFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {

        MessageOf<tuple<int,int,int>>* msg = static_cast<MessageOf<tuple<int,int,int>>*>(_msg.get());
        tuple<int,int,int> msgData = *msg->getData();

        console << "rec. Flood (" << std::get<0>(msgData) << "," << std::get<1>(msgData) << ") from " << sender->getConnectedBlockId() << "\n";
        console << "block: " << getId() << " has a distance of: " << distance << " and as parent: " << parent << "\n";
        if ( (parent==nullptr || std::get<0>(msgData)<distance || currentRound<std::get<1>(msgData)) ) {
            distance=std::get<0>(msgData);
            maxd=distance;
            console << maxd <<"\n";
            currentRound=std::get<1>(msgData);
            parent=sender;
            string str="distance(";
            str+=to_string(distance+1)+","+to_string(currentRound)+")";
            nbWaitedAnswers=sendMessageToAllNeighbors(str.c_str(),new MessageOf<tuple<int,int,int>>(BROADCAST_MSG_ID,make_tuple(distance+1,currentRound,maxd)),1000,100,1,sender);
            if (nbWaitedAnswers==0) {
                //End of line blocks enter this
                sendMessage("ack2parent",new MessageOf<int>(ACKNOWLEDGE_MSG_ID, maxd),parent,1000,100);//sends a message to one connected block
            }
        } 
        else {
                //blocks that have already had their children talked to enter this
                sendMessage("ack2sender",new MessageOf(ACKNOWLEDGE_MSG_ID, -1),sender,1000,100);
        }
    }

    void MyAppCode::myAcknowledgeFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
        MessageOf<int>* msg = static_cast<MessageOf<int>*>(_msg.get());
        int msgData = *msg->getData();
        if(msgData > maxd) {
            maxd = msgData;
            maxInt = sender;
        }
        console << "At block: " << getId() << " distance is:" << distance << "\n";
        nbWaitedAnswers--;
        console << "rec. Ack(" << nbWaitedAnswers << ") from " << sender->getConnectedBlockId() << "\n";
        if (nbWaitedAnswers==0) {
            if (parent==nullptr) {
                console << "the maxd is:" << maxd << "\n";
                if(currentRound > 2) return;
                sendMessage("toMax", new Message(TOMAX_MSG_ID), maxInt, 1000, 100);
                // nbWaitedAnswers=sendMessageToAllNeighbors("distance(1,2)",new MessageOf<tuple<int,int,int>>(BROADCAST_MSG_ID,make_tuple(distance+1,currentRound,maxd)),1000,100,0);
            } else {
                sendMessage("ack2parent",new MessageOf(ACKNOWLEDGE_MSG_ID, maxd),parent,1000,100);
                parent = nullptr;
            }
        }
    }

    void MyAppCode::toMaxFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
        if(currentRound>1) {
            if(isA){
                setColor(RED);
                if(maxInt != nullptr) {
                    sendMessage("toMax", new Message(TOMAX_MSG_ID), maxInt, 1000, 100);
                }
            }
            else{
                setColor(WHITE);
                console << "We are running in round " << currentRound << "the sender is:" << sender->getConnectedBlockId() << "\n";
                if(distance == maxd / 2) {
                    setColor(BROWN);
                }
                if(maxInt != nullptr) {
                    sendMessage("toMax", new Message(TOMAX_MSG_ID), maxInt, 1000, 100);
                } else {
                    setColor(BLUE);
                    currentRound++;
                }
            }
        }
        if(currentRound==1){
            if(maxInt != nullptr) {
                sendMessage("toMax", new Message(TOMAX_MSG_ID), maxInt, 1000, 100);
            } else {
                setColor(YELLOW);
                parent = nullptr;
                currentRound++;
                distance=0;
                nbWaitedAnswers=sendMessageToAllNeighbors("distance(1,2)",new MessageOf<tuple<int,int,int>>(BROADCAST_MSG_ID,make_tuple(distance+1,currentRound,maxd)),1000,100,0);
            }
        }
    }

    void MyAppCode::parseUserBlockElements(TiXmlElement *config) {
        const char *attr = config->Attribute("isA");
        isA = (attr?Simulator::extractBoolFromString(attr):false);
        if (isA) {
            std::cout << getId() << " is A!" << std::endl; // complete with your code
        }
    }

    void MyAppCode::onUserKeyPressed(unsigned char c, int x, int y) {
        switch (c) {
            case 'a' : // update with your code
                std::cout << "key a" << endl;
                break;
            case 'd' : 
                break;
        }
    }

    void MyAppCode::onTap(int face) {
        std::cout << "Block 'tapped':" << getId() << std::endl;
        module->setColor(RED);
        currentRound++;
        distance=0;
        nbWaitedAnswers=sendMessageToAllNeighbors("distance(1,2)",new MessageOf<tuple<int,int>>(BROADCAST_MSG_ID,make_tuple(distance+1,currentRound)),1000,100,0); // complete with your code here
    }