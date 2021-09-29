#include <iostream>

#include <Antilatency.InterfaceContract.LibraryLoader.h>
#include <Antilatency.DeviceNetwork.h>
#if defined(__linux__)
#include <dlfcn.h>
#include <filesystem>
#endif

#include <thread>
#include <chrono>
#include <mutex>
//---------------------------------
//	OSC
//---------------------------------
#include <string>
#include "OscReceivedElements.h"
#include "OscPacketListener.h"
#include "OscOutboundPacketStream.h"
#include "UdpSocket.h"
#include "IpEndpointName.h"


std::mutex mtx;

void sendOSCMessage(int id, float x, float y, float z, float rx, float ry, float rz, float rw, int state, float stability)
{
    // Set IPAddress and Port
    const std::string ipAddress = "127.0.0.1";
    const int port = 8000;

    UdpTransmitSocket transmitSocket(IpEndpointName(ipAddress.c_str(), port));
    //Buffer
    char buffer[6144];
    osc::OutboundPacketStream p(buffer, 6144);
    p << osc::BeginBundleImmediate
        //Head
        << osc::BeginMessage("/position") << id << x << y << z << rx << ry << rz << rw << state << stability << osc::EndMessage
        << osc::EndBundle;
    transmitSocket.Send(p.Data(), p.Size());
}


Antilatency::DeviceNetwork::NodeHandle getIdleTrackingNode(Antilatency::DeviceNetwork::INetwork network, Antilatency::Alt::Tracking::ITrackingCotaskConstructor altTrackingCotaskConstructor) {
    // Get all currently connected nodes, that supports alt tracking task.
    std::vector<Antilatency::DeviceNetwork::NodeHandle> altNodes = altTrackingCotaskConstructor.findSupportedNodes(network);
    if (altNodes.size() == 0) {
        std::cout << "No nodes with Alt Tracking Task support found" << std::endl;
        return Antilatency::DeviceNetwork::NodeHandle::Null;
    }

    // Return first idle node.
    for (auto node : altNodes) {
        if (network.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle) {
            return node;
        }
    }

    std::cout << "No idle nodes with Alt Tracking Task support found" << std::endl;
    return Antilatency::DeviceNetwork::NodeHandle::Null;
}

std::vector<Antilatency::DeviceNetwork::NodeHandle> getIdleTrackingNodes(Antilatency::DeviceNetwork::INetwork network, Antilatency::Alt::Tracking::ITrackingCotaskConstructor altTrackingCotaskConstructor) {
    // Get all currently connected nodes, that supports alt tracking task.
    std::vector<Antilatency::DeviceNetwork::NodeHandle> altNodes = altTrackingCotaskConstructor.findSupportedNodes(network);
    std::vector< Antilatency::DeviceNetwork::NodeHandle> returnNodes;
    if (altNodes.size() == 0) {
        std::cout << "No nodes with Alt Tracking Task support found" << std::endl;
        return returnNodes;
    }

    // Pushback all available node.
    for (auto node : altNodes) {
        if (network.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle) {
            returnNodes.push_back(node);
        }
    }

    std::cout << "Idle nodes with Alt Tracking Task support found: Count ----- " << returnNodes.size() << std::endl;
    return returnNodes;
}

void getTrackingInfo(Antilatency::Alt::Tracking::ITrackingCotask& altTrackingCotask, std::string tag, Antilatency::Math::floatP3Q placement) {
    if (altTrackingCotask != nullptr) {
        while (altTrackingCotask != nullptr)
        {
            if (altTrackingCotask.isTaskFinished()) {
                std::cout << "Tracking task finished: " << tag << std::endl;
                return;
            }

            Antilatency::Alt::Tracking::State state = altTrackingCotask.getExtrapolatedState(placement, 0.03f);
            // id   
            int id = std::stoi(tag);
            // Pose
            float posx = state.pose.position.x;
            float posy = state.pose.position.y;
            float posz = state.pose.position.z;
            float rx = state.pose.rotation.x;
            float ry = state.pose.rotation.y;
            float rz = state.pose.rotation.z;
            float rw = state.pose.rotation.w;

            int curState = static_cast<int32_t>(state.stability.stage);
            float stability = state.stability.value;

            // -------------------------------------------------
            // Info
            // -------------------------------------------------
            //std::cout << "State: " << id << std::endl;
            //std::cout << "\tPose:" << std::endl;
            //std::cout << "\t\tPosition: x: " << posx << ", y: " << posy << ", z: " << posz << std::endl;
            //std::cout << "\t\tRotation: x: " << rx << ", y: " << ry << ", z: " << rz << ", w: " << rw << std::endl;
               
            //std::cout << "\tStability:" << std::endl;
            //std::cout << "\t\tStage: " << static_cast<int32_t>(state.stability.stage) << std::endl;
            //std::cout << "\t\tValue: " << state.stability.value << std::endl;
            //std::cout << "\tVelocity:" << state.velocity.x << ", y: " << state.velocity.y << ", z: " << state.velocity.z << std::endl;
            //std::cout << "\tLocalAngularVelocity: x:" << state.localAngularVelocity.x << ", y: " << state.localAngularVelocity.y << ", z: " << state.localAngularVelocity.z << std::endl << std::endl;
            
            // -------------------------------------------------

            // Send OSC
            mtx.lock();
            sendOSCMessage(id, posx, posy, posz, rx, ry, rz, rw, curState, stability);
            mtx.unlock();
            
            // Wait
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(30));
        }
    }
    else {
        std::cout << "Failed to start tracking task on node" << std::endl;
    }
}








// ------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    if(argc != 3){
        std::cout << "Wrong arguments. Pass environment data string as first argument and placement data as second.";
        return 1;
    }
	#if defined(__linux__)
        Dl_info dlinfo;
        dladdr(reinterpret_cast<void*>(&main), &dlinfo);
        std::string path = std::filesystem::path(dlinfo.dli_fname).parent_path();
        std::string libNameADN = path + "/libAntilatencyDeviceNetwork.so";
        std::string libNameTracking = path + "/libAntilatencyAltTracking.so";
        std::string libNameEnvironmentSelector = path + "/libAntilatencyAltEnvironmentSelector.so";
	#else
		std::string libNameADN = "AntilatencyDeviceNetwork";
		std::string libNameTracking = "AntilatencyAltTracking";
		std::string libNameEnvironmentSelector = "AntilatencyAltEnvironmentSelector";
	#endif
	
    // Load the Antilatency Device Network library
    Antilatency::DeviceNetwork::ILibrary deviceNetworkLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::DeviceNetwork::ILibrary>(libNameADN.c_str());
    if (deviceNetworkLibrary == nullptr) {
        std::cout << "Failed to get Antilatency Device Network Library" << std::endl;
        return 1;
    }

    // Load the Antilatency Alt Tracking library
    Antilatency::Alt::Tracking::ILibrary altTrackingLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Tracking::ILibrary>(libNameTracking.c_str());
    if (altTrackingLibrary == nullptr) {
        std::cout << "Failed to get Antilatency Alt Tracking Library" << std::endl;
        return 1;
    }

    // Load the Antilatency Alt Environment Selector library
    Antilatency::Alt::Environment::Selector::ILibrary environmentSelectorLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Environment::Selector::ILibrary>(libNameEnvironmentSelector.c_str());
    if (environmentSelectorLibrary == nullptr) {
        std::cout << "Failed to get Antilatency Alt Environment Selector Library" << std::endl;
        return 1;
    }

    // Create a device network filter and then create a network using that filter.
    Antilatency::DeviceNetwork::IDeviceFilter filter = deviceNetworkLibrary.createFilter();
    filter.addUsbDevice(Antilatency::DeviceNetwork::Constants::AllUsbDevices);
    Antilatency::DeviceNetwork::INetwork network = deviceNetworkLibrary.createNetwork(filter);
    if (network == nullptr) {
        std::cout << "Failed to create Antilatency Device Network" << std::endl;
        return 1;
    }
    std::cout << "Antilatency Device Network created" << std::endl;

    // Get environment serialized data.
    const std::string environmentData = argv[1];
    // Get placement serialized data.
    const std::string placementData = argv[2];

    // Create environment object from the serialized data.
    const Antilatency::Alt::Environment::IEnvironment environment = environmentSelectorLibrary.createEnvironment(environmentData);
    if (environment == nullptr) {
        std::cout << "Failed to create environment" << std::endl;
        return 1;
    }

    // Create placement from the serialized data.
    const Antilatency::Math::floatP3Q placement = altTrackingLibrary.createPlacement(placementData);

    // Create alt tracking cotask constructor to find tracking-supported nodes and start tracking task on node.
    Antilatency::Alt::Tracking::ITrackingCotaskConstructor altTrackingCotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();
    if (altTrackingCotaskConstructor == nullptr) {
        std::cout << "Failed to create Antilatency Alt Tracking Cotask Constructor" << std::endl;
        return 1;
    }

    

    // Each time the device network is changed due to connection or disconnection of a device that matches the device filter of the network,
    // or start or stop of a task on any network device, the network update id is incremented by 1. 
    uint32_t prevUpdateId = 0;

    while (network != nullptr) {
        // Check if the network has been changed.
        const uint32_t currentUpdateId = network.getUpdateId();
        if (prevUpdateId != currentUpdateId) {
            prevUpdateId = currentUpdateId;
            std::cout << "--- Device network changed, update id: " << currentUpdateId << " ---" << std::endl;
            std::vector<std::thread> workers;
            // Get all idle nodes that supports tracking task.
            const std::vector<Antilatency::DeviceNetwork::NodeHandle> trackingNodes = getIdleTrackingNodes(network, altTrackingCotaskConstructor);
            std::cout << trackingNodes.size() << std::endl;
            for (auto trackingNode : trackingNodes) {
                if (trackingNode != Antilatency::DeviceNetwork::NodeHandle::Null) {
                    // Start tracking task on node.
                    Antilatency::Alt::Tracking::ITrackingCotask altTrackingCotask = altTrackingCotaskConstructor.startTask(network, trackingNode, environment);
                    
                    // Tag
                    std::string tag;
                    tag = network.nodeGetStringProperty(trackingNode, "Tag");
                    std::cout << "Start:" << tag << std::endl;
                    workers.emplace_back(getTrackingInfo, altTrackingCotask, tag, placement);
                    //getTrackingInfo(altTrackingCotask, tag, placement);
                }
            }
            for (auto& worker : workers) {
                worker.join();
                std::cout << "thread join" << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(500));
    }
    
    return 0;
}
