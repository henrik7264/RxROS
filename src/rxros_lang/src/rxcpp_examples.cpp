//
// Created by hl on 1/13/19.
//
#include <Scheduler.h> // Bosma::Scheduler
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <numeric>
#include <thread>
#include <mutex>
using namespace std;

#include <rxcpp/rx.hpp>
#include <rxcpp/schedulers/rx-currentthread.hpp>
#include <rxcpp/schedulers/rx-eventloop.hpp>
#include <rxcpp/schedulers/rx-immediate.hpp>
#include <rxcpp/schedulers/rx-newthread.hpp>
#include <rxcpp/schedulers/rx-runloop.hpp>
#include <rxcpp/operators/rx-reduce.hpp>
#include <rxcpp/operators/rx-filter.hpp>
#include <rxcpp/operators/rx-map.hpp>
#include <rxcpp/operators/rx-tap.hpp>
#include <rxcpp/operators/rx-concat_map.hpp>
#include <rxcpp/operators/rx-flat_map.hpp>
#include <rxcpp/operators/rx-concat.hpp>
#include <rxcpp/operators/rx-merge.hpp>
#include <rxcpp/operators/rx-repeat.hpp>
#include <rxcpp/operators/rx-publish.hpp>
#include <rxcpp/operators/rx-ref_count.hpp>
namespace Rx {
    using namespace rxcpp;
    using namespace rxcpp::sources;
    using namespace rxcpp::operators;
    using namespace rxcpp::util;
}
using namespace Rx;


class Examples
{
private:
public:
    Examples() {}
    virtual ~Examples() {}
    void lambdaFunction1();
    void lambdaFunction2();
    void lambdaFunction3();
    void lambdaFunction4();
    void rxCreate();
    void rxRange();
    void rxMap();
    void rxMerge();
    void rxConcat();
    void rxPipe();
    void rxMerge2();

    void stdScheduler();
    void rxTime();
    void rxInterval();
    void rxScheduler1();
    void rxScheduler2();
    void rxObserveOnScheduler();
    void rxSubscribeOnScheduler();
};


//--------------------------------------------------------------------------------------
void Examples::lambdaFunction1()
{
    auto numbers = vector<int>{ 10, 23, -33, 15, -7, 60, 80};
    // Lambda expression. Returns true if value is positive else false.
    auto isPositveFunc = [](int const val) {return val > 0; };
    // Count the number of positive values
    auto cnt = count_if(begin(numbers), end(numbers), isPositveFunc);
    cout << cnt << endl;
}

//--------------------------------------------------------------------------------------
void Examples::lambdaFunction2()
{
    auto numbers = vector<int>{ 10, 23, -33, 15, -7, 60, 80};
    // Lambda expression take two arguments of any type and returns the sum.
    auto accum = std::accumulate(std::begin(numbers), std::end(numbers), 0, [](auto const accSum, auto const val) {return accSum + val;});
    cout << accum << endl;
}

//--------------------------------------------------------------------------------------
// Functor to add two numbers of a given type
template <typename T>
struct addition {
    // This operator overloading enables calling operator function () on objects of addition.
    T operator () (const T& a,  const T& b ) { return a + b; }
};

void Examples::lambdaFunction3()
{
    double numbers[3] = {1.0, 2.0, 4.0};
    double sum = 0;

    // Functors vs Lambda expressions.
    sum = accumulate(numbers, numbers+3, 0.0, addition<double>());
    cout << "sum = " << sum << endl;
    sum = accumulate(numbers, numbers+3, 0.0, [] (const double& accSum, const double& val ) {return accSum +val;});
    cout << "sum = " << sum << endl;
}

//--------------------------------------------------------------------------------------
// Generic higher order function that takes two functions f and g and returns f(g())
template <typename F, typename G>
auto Compose(F&& f, G&& g) {
    // returns a lambda expression of applying f(g(x)).
    // The = means that the lambda expression will access objects from its enclosing scope by value.
    return [=](auto x) { return f(g(x)); };
}

//Generic higher order function that takes a number of functions and
// recursive applies each function on the the result of previous function, i.e. f(g(h(...)))
template <typename F, typename... R>
auto Compose(F&& f, R&&... r){
    return [=](auto x) { return f(Compose(r...)(x)); };
}

// Function that takes takes three arguments (int) and return the sum of them.
// Observe that the lambda expressions make use of objects from the enclosing scope.
auto CurriedAdd3(int x) {
    return [x](int y) { //capture x
        return [x, y] (int z) { return x + y + z; };
    };
};

void Examples::lambdaFunction4()
{
    // Compose two functions together
    auto val = Compose(
            [](int const a) {return to_string(a); },
            [](int const a) {return a * a; }) ( 4 ); // val = "16"
    cout << val << std::endl; //should print 16

    //Compose a set of function together
    auto func = Compose(
            [](int const n) {return to_string(n); },
            [](int const n) {return n * n; },
            [](int const n) {return n + n; },
            [](int const n) {return abs(n); });
    cout << func(5) << endl;

    // Invoke the Curried function
    auto p = CurriedAdd3(4)(5)(6);
    cout << p << endl;
}


//--------------------------------------------------------------------------------------
void Examples::rxCreate()
{
    auto ints = rxcpp::observable<>::create<int> (
        [](rxcpp::subscriber<int> s)
        {
            s.on_next(1);
            s.on_next(2);
            s.on_completed();
        });

    ints.subscribe(
        [](int v) {cout << "OnNext: " << v << endl;},
        []() {cout << "OnCompleted" << endl;});
}

//--------------------------------------------------------------------------------------
void Examples::rxRange()
{
    auto values = rxcpp::observable<>::range(1, 10);
    values.subscribe(
        [](int v) {cout << "OnNext: " << v << endl;},
        []() {cout << "OnCompleted" << endl;});
}


//--------------------------------------------------------------------------------------
void Examples::rxMap()
{
    auto ints = rxcpp::observable<>::range(1, 10).map([](int n) { return n * n; });
    ints.subscribe(
        [](int v) { cout << "OnNext: " << v << endl; },
        []() { cout << "OnCompleted" << endl; });
}


//--------------------------------------------------------------------------------------
void Examples::rxMerge()
{
    auto o1 = rxcpp::observable<>::range(1, 3);
    auto o2 = rxcpp::observable<>::from(4, 5, 6);
    auto values = o1.merge(o2);
    values.subscribe(
        [](int v) {cout << "OnNext: " <<  v << endl;},
        []() {cout << "OnCompleted" << endl;});
}


//--------------------------------------------------------------------------------------
void Examples::rxConcat()
{
    auto o1 = rxcpp::observable<>::range(1, 3);
    auto o2 = rxcpp::observable<>::from(4, 5, 6);
    auto values = o1.concat(o2);
    values.subscribe(
        [](int v) {cout << "OnNext: " <<  v << endl;},
        []() {cout << "OnCompleted" << endl;});
}


//--------------------------------------------------------------------------------------
void Examples::rxPipe()
{
    auto ints = rxcpp::observable<>::range(1,10) | Rx::map([](int n) { return n * n; });
    ints.subscribe(
        [](int v){cout << "OnNext: " << v << endl;},
        [](){cout << "OnCompleted" << endl;});
}


//--------------------------------------------------------------------------------------
void Examples::rxMerge2()
{
    auto o1 = rxcpp::observable<>::timer(chrono::milliseconds(15)).map([](int) { return 1; });
    auto o2 = rxcpp::observable<>::error<int>(runtime_error("Error from source"));
    auto o3 = rxcpp::observable<>::timer(chrono::milliseconds(5)).map([](int) { return 3; });
    auto base = rxcpp::observable<>::from(o1.as_dynamic(), o2, o3);
    auto values = base.merge();

    values.subscribe(
            [](int v) { cout << "OnNext: " << v << endl; },
            [](exception_ptr e) { cout << "OnError: " << Rx::what(e) << endl; },
            []() { cout << "OnCompleted" << endl; });
}



//--------------------------------------------------------------------------------------
void Examples::stdScheduler()
{
    Bosma::Scheduler scheduler; // default 4 threads.

    // every second call
    scheduler.every(std::chrono::seconds(1), []() { std::cout << "in one minute" << std::endl; });

    // in one minute
    scheduler.in(std::chrono::minutes(1), []() { std::cout << "in one minute" << std::endl; });

    // Time formats supported:
    // %Y/%m/%d %H:%M:%S, %Y-%m-%d %H:%M:%S, %H:%M:%S
    // With only a time given, it will run tomorrow if that time has already passed.
    // But with a date given, it will run immediately if that time has already passed.
    scheduler.at("2019-02-15 17:00:00", []() { std::cout << "at a specific time." << std::endl; });
}


//--------------------------------------------------------------------------------------
void Examples::rxTime()
{
    auto start = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
    auto values = rxcpp::observable<>::timer(start);
    values.subscribe(
        [](int v) { printf("OnNext: %d\n", v); },
        []() { printf("OnCompleted\n"); });


    auto period = std::chrono::milliseconds(10);
    auto values2 = rxcpp::observable<>::timer(period);
    values2.subscribe(
        [](int v) { printf("OnNext: %d\n", v); },
        []() { printf("OnCompleted\n"); });


    auto o1 = rxcpp::observable<>::timer(std::chrono::milliseconds(15)).map([](int) { return 1; });
    auto o2 = rxcpp::observable<>::timer(std::chrono::milliseconds(10)).map([](int) { return 2; });
    auto o3 = rxcpp::observable<>::timer(std::chrono::milliseconds(5)).map([](int) { return 3; });
    auto values3 = rxcpp::observable<>::from(o1.as_dynamic(), o2, o3).merge();
    values3.subscribe(
        [](int v) { printf("OnNext: %d\n", v); },
        []() { printf("OnCompleted\n"); });

}

//--------------------------------------------------------------------------------------
void Examples::rxInterval()
{
    auto values = rxcpp::observable<>::interval(std::chrono::milliseconds(1000)).take(10);
    values.subscribe(
        [](int v){printf("OnNext: %d\n", v);},
        [](){printf("OnCompleted\n");});


    auto start = std::chrono::steady_clock::now() + std::chrono::milliseconds(10000);
    auto period = std::chrono::milliseconds(1000);
    auto values2 = rxcpp::observable<>::interval(start, period).take(10);
    values2.subscribe(
        [](int v){printf("OnNext: %d\n", v);},
        [](){printf("OnCompleted\n");});
}


//--------------------------------------------------------------------------------------
// RxCpp operates with:
// * Scheduler
// * Worker
// * Coordination
// * Coordinator
// * Schedulable
// * Timeline
//
// * A Scheduler has a timeline.
// * A Scheduler can create a lot of workers in the timeline.
// * A Worker owns a queue of Schedulabes in the timeline.
// * A Schedulable owns a function (called an Action) and has a lifetime.
// * A Coordination is a factory for Coordinators and has a scheduler.
// * A Coordinator has a Worker and is factory for
//     * Coordinated Schedulers
//     * Coordinated Observers and Subscribers.
//
// * In RxCpp, All operators that take multiple streams as input or
//   deal with tasks that involves time take a Coordination as a parameter.

// * Some of the Coordination functions using a particular Scheduler are:
//     * identity_immediate()
//     * identity_current_thread()
//     * identity_same_worker(worker w)
//     * serialize_event_loop();
//     * serialize_new_thread();
//     * serialize_same_worker(worker w);
//     * observe_on_event_loop();
//     * observe_on_new_thread();

void printThreadId(int val = 0)
{
    static std::mutex mutex;
    mutex.lock();
    cout << "Current Thread id: " << this_thread::get_id() << ", " << val << endl;
    mutex.unlock();
}

void Examples::rxScheduler1()
{
    // Get a Coordination
    auto coordination = rxcpp::serialize_new_thread();

    // Create a Worker instance
    auto worker = coordination.create_coordinator().get_worker();

    // Create an Action
    auto action = rxcpp::schedulers::make_action([](const rxcpp::schedulers::schedulable&) {printThreadId(1234);});

    // Create a Schedulable
    auto scheduled = rxcpp::schedulers::make_schedulable(worker, action);

    scheduled.schedule();
}


//--------------------------------------------------------------------------------------
void Examples::rxScheduler2()
{
    // Create an Observable
    auto values = rxcpp::observable<>::interval(std::chrono::milliseconds(50)).take(5);

    // Coordination object
    auto coordination = rxcpp::serialize_new_thread();

    // Retrieve the worker
    auto worker = coordination.create_coordinator().get_worker();

    // Subscribe from the beginning
    worker.schedule(
        [&](const rxcpp::schedulers::schedulable&) {
            values.subscribe(
                [](long v){ printThreadId(v);},
                [](){ printThreadId(2345);});});

    // Wait before subscribing
    worker.schedule(coordination.now() + std::chrono::milliseconds(100),
        [&](const rxcpp::schedulers::schedulable&) {
            values.subscribe(
                [](long v){ printThreadId(v*v);},
                [](){ printThreadId(2345);});});

    // Add blocking subscription to see results
    values.as_blocking().subscribe();
}

//--------------------------------------------------------------------------------------
void Examples::rxObserveOnScheduler()
{
    //Print the main thread id
    printThreadId();

    //We are using observe_on here
    //The Map will use the main thread
    //Subscribed lambda will use a new thread
    rxcpp::observable<>::range(0,15).
        map([](int i) {
            printThreadId(i);  // will execute in main thread.
            return i;}).
        take(5).
        observe_on(rxcpp::synchronize_new_thread()).
        subscribe(
            [&](int i) {printThreadId(i*i);}); // will execute in new thread.

    //Wait for Two Seconds
    rxcpp::observable<>::timer(std::chrono::milliseconds(2000)).subscribe();
}


void Examples::rxSubscribeOnScheduler()
{
    //Print the main thread id
    printThreadId();

    //We are using subscribe_on here
    //The Map and subscribed lambda will
    //use the secondary thread
    rxcpp::observable<>::range(0, 15).
        map([](int i) {
            printThreadId(i);
            return i;}).
        take(5).
        subscribe_on(rxcpp::synchronize_new_thread()). // will all execute in new thread
        subscribe(
                [&](int i) {printThreadId(i*i);});

    //Wait for Two Seconds
    rxcpp::observable<>::timer(std::chrono::milliseconds(2000)).subscribe();
}

//--------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    Examples examples;
    examples.lambdaFunction1();
    examples.lambdaFunction2();
    examples.lambdaFunction3();
    examples.lambdaFunction4();

    examples.rxCreate();
    examples.rxRange();
    examples.rxMap();
    examples.rxMerge();
    examples.rxConcat();
    examples.rxPipe();
    examples.rxMerge2();

    examples.stdScheduler();
    examples.rxTime();
    examples.rxInterval();
    examples.rxScheduler1();
    examples.rxScheduler2();
    examples.rxObserveOnScheduler();
    examples.rxSubscribeOnScheduler();
}
