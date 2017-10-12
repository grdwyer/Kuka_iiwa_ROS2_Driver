//
// Created by george on 26/09/17.
//

#include <ros/ros.h>
#include <iiwa_fri_ros/IiwaHWInterface.h>
#include <thread>

class Reader{
private:
    std::shared_ptr<IiwaState> state_;

public:
    Reader(std::shared_ptr<IiwaState> state){
        state_ = state;
    }

    void read_values( ){
        std::cout << "Command position: ";
        for (const auto a: state_->command_position_){
            std::cout << a << ", ";
        }
        std::cout << std::endl;

    }
};

class Adder{
private:
    std::shared_ptr<IiwaState> state_;
    std::array<double, 7> val_;

public:
    Adder(std::shared_ptr<IiwaState> state){
        state_ = state;
        val_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    void add_values(){
        std::cout << "Iterating val " << std::endl;
        for (auto a: val_){
            a++;
        }
        std::cout << "Copying accross to state" << std::endl;
        std::memcpy(state_->command_position_.data(), val_.data(), 7);

    }
};


int main (int argc, char** argv)
{

    auto state = std::make_shared<IiwaState>();

    auto read = Reader(state);
    auto write = Adder(state);

    for(int i = 0; i < 5; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        read.read_values();
        write.add_values();

    }

}