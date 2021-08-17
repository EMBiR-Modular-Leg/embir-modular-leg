#include <vector>

std::vector<double> binomial_mult( int n, std::vector<double> p );
std::vector<double> trinomial_mult( int n, std::vector<double> b, std::vector<double> c );

std::vector<double> dcof_bwlp( int n, double fcf );
std::vector<double> dcof_bwhp( int n, double fcf );
std::vector<double> dcof_bwbp( int n, double f1f, double f2f );
std::vector<double> dcof_bwbs( int n, double f1f, double f2f );

std::vector<int> ccof_bwlp( int n );
std::vector<int> ccof_bwhp( int n );
std::vector<int> ccof_bwbp( int n );
std::vector<double> ccof_bwbs( int n, double f1f, double f2f );

double sf_bwlp( int n, double fcf );
double sf_bwhp( int n, double fcf );
double sf_bwbp( int n, double f1f, double f2f );
double sf_bwbs( int n, double f1f, double f2f );
