//
// Created by hl on 1/13/19.
//
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <numeric>
using namespace std;

#include <rxcpp/rx.hpp>
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
    void rxTimer1();
    void rxTimer2();
    void rxScheduler1();
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
void Examples::rxTimer1()
{
    auto start = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
    auto values = rxcpp::observable<>::timer(start);
    values.subscribe(
        [](int v){printf("OnNext: %d\n", v);},
        [](){printf("OnCompleted\n");});

    auto period = std::chrono::milliseconds(10);
    auto values2 = rxcpp::observable<>::timer(period);
    values2.subscribe(
        [](int v){printf("OnNext: %d\n", v);},
        [](){printf("OnCompleted\n");});
}


//--------------------------------------------------------------------------------------
void Examples::rxTimer2()
{
    auto o1 = rxcpp::observable<>::timer(std::chrono::milliseconds(15)).map([](int) { return 1; });
    auto o2 = rxcpp::observable<>::timer(std::chrono::milliseconds(10)).map([](int) { return 2; });
    auto o3 = rxcpp::observable<>::timer(std::chrono::milliseconds(5)).map([](int) { return 3; });
    auto values = rxcpp::observable<>::from(o1.as_dynamic(), o2, o3).merge();
    values.subscribe(
        [](int v) { printf("OnNext: %d\n", v); },
        []() { printf("OnCompleted\n"); });
}


//--------------------------------------------------------------------------------------
void Examples::rxScheduler1()
{
    auto scheduler = rxcpp::observe_on_new_thread();
    auto period = std::chrono::milliseconds(1);
    auto values = rxcpp::observable<>::timer(period, scheduler).
        finally([](){printf("The final action\n");});

    values.as_blocking().subscribe(
        [](int v){printf("OnNext: %d\n", v);},
        [](){printf("OnCompleted\n");});
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

    examples.rxTimer1();
    examples.rxTimer2();
    examples.rxScheduler1();
}
