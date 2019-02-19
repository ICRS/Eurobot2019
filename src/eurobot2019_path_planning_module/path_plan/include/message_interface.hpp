#ifndef __MESSAGE_INTERFACE_HPP__
#define __MESSAGE_INTERFACE_HPP__

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <string>
#include <mutex>
#include <thread>

// Simplification
typedef std_msgs::Empty E;

// Forward Declarations
template <class Tpub, class Tsub>
class MessageInterface;

// Easier to use versions - functions are all the same, 
// but it's constructed all for you. To make a multi 
// interface you gotta use the big boi down there :/

template <class Tpub>
class MessagePublisherInterface :public MessageInterface<Tpub, E> {
public:
    MessagePublisherInterface(float frequency, std::string topic)
            :MessageInterface<Tpub, E>(frequency, topic, 0, "") {};
};

template <class Tsub>
class MessageSubscriberInterface :public MessageInterface<E, Tsub> {
public:
    MessageSubscriberInterface(float frequency, std::string topic)
            :MessageInterface<E, Tsub>(0, "", frequency, topic) {};
};

// Allows creation of one or the other but not both
// Both is possible via full constructor
enum MessageInterfaceType {
    PUBLISHER,
    SUBSCRIBER
};
    
template <class Tpub, class Tsub>
class MessageInterface {
public:
    // Constructor can create any type of interface
    MessageInterface(float publishing_frequency,
                     std::string publishing_topic,
                     float subscribing_frequency,
                     std::string subscribing_topic) {
        initialise(publishing_frequency, publishing_topic,
                   subscribing_frequency, subscribing_topic);
    };

    // Constructs publisher or subscriber, not both
    MessageInterface(float frequency,
                     std::string topic,
                     MessageInterfaceType class_type) {
        if(class_type == MessageInterfaceType::PUBLISHER)
            initialise(frequency, topic, 0, "");
        else  
            initialise(0, "", frequency, topic); 
    };

    // Destructor
    virtual ~MessageInterface() {
        // Stop loop
        running_ = false;

        // Wait for thread to finish
        thread_.join();
    }
    
    // Set the message to be published. It will be published repeatedly
    // until a new one is set
    void set_msg(Tpub msg) {
        // Accquire lock on the message to prevent race conditions
        std::lock_guard<std::mutex> lock(pub_mutex_);

        current_pub_msg_ = msg;
    };

    // Get the most recently received 
    Tsub get_msg() {
        // Accquire lock on the message to prevent race conditions
        std::lock_guard<std::mutex> lock(sub_mutex_);

        return current_sub_msg_;
    };

protected:
    void initialise(float publishing_frequency,
                    std::string publishing_topic,
                    float subscribing_frequency,
                    std::string subscribing_topic) {
        // Set to manage my own callback queue
        nh_.setCallbackQueue(&callback_queue_);

        if(publishing_frequency) {
            is_publisher_ = true;
            publisher_ = nh_.advertise<Tpub>(publishing_topic, 
                                             publishing_frequency);
        }

        if(subscribing_frequency) {
            subscriber_ = nh_.subscribe(subscribing_topic, 
                                        subscribing_frequency,
                                        &MessageInterface::receive_msg,
                                        this);
        }

        running_ = true;
        
        // Initialise thread via lambda
        thread_ = std::thread([this, publishing_frequency]() {
            this->loop(publishing_frequency);
        });
    };

    // Actually publish
    void publish() {
        if(is_publisher_) {
            // Accquire lock on the message to prevent race conditions
            std::lock_guard<std::mutex> lock(pub_mutex_);
            
            publisher_.publish(current_pub_msg_);
        }
    };

    // Main loop
    void loop(float frequency) {
        // Control publishing rate
        ros::Rate loop_rate(frequency);

        while(ros::ok() && running_) {
            publish();
            callback_queue_.callAvailable(ros::WallDuration());
            loop_rate.sleep();
        }
    };

    // Callback for receiving a message
    void receive_msg(const Tsub &msg) {
        // Accquire lock on the message to prevent race conditions
        std::lock_guard<std::mutex> lock(sub_mutex_);
        
        current_sub_msg_ = msg;
    };

    // Thread to run the publishing
    std::thread thread_;
    
    // Control the thread
    bool running_;
    
    // Current message to be published
    Tpub current_pub_msg_;

    // Most recently received message
    Tsub current_sub_msg_;
    
    // Mutexes to prevent data from being changed mid-publish/subscribe
    std::mutex pub_mutex_;
    std::mutex sub_mutex_;
    
    // Personal NodeHandle and queue to allow for threaded callbacks
    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue_;

    // The actual publisher and subscriber
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;

    // Type
    bool is_publisher_;
};

#endif // __MESSAGE_INTERFACE_HPP__
