//
// Created by hl on 1/13/19.
//
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <numeric>
#include <rxcpp/rx.hpp>
using namespace std;


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

    auto ints = rxcpp::observable<>::range(1,10) | map([](int n) {return n * n;});
    ints.subscribe(
        [](int v){cout << "OnNext: " << v << endl;},
        [](){cout << "OnCompleted" << endl;});
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
}
