#include <iostream>
#include <thread>
#include <memory>

using namespace std;

class MyClass {
public:
    thread my_thread;
    int id;

    MyClass(int input_id){
        id = input_id;
    }

    void myFunction() {
        cout << "Hello from MyClass!" << id << endl;
    }

    void runThread() {
        my_thread = thread(&MyClass::myFunction, this);
    }

    void joinThread() {
        my_thread.join();
    }
};

int main() {
    shared_ptr<MyClass> my_shared_ptr = make_shared<MyClass>(10);

    my_shared_ptr->runThread();

    my_shared_ptr->joinThread();

    return 0;
}