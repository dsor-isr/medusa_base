// g++ digital_filter_design.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <tuple>
#include <complex>


//------------------------------------------------------------------

std::vector<std::complex<double>> computeSignalDev()

{

    int size = 1000;
    double dt = 1.0 / size;
    std::vector<std::complex<double>> yy_signal;
    std::vector<double> tt;

    double f1 = 2.0;
    double f2 = 50.0;
    double A1 = 1.0;
    double A2 = 0.2;

    for (double t = 0.0; t <= 1.0; t = t + dt)
    {

        auto currentSample = std::complex<double>(A1 * std::sin(2 * M_PI * f1 * t) + A2 * std::sin(2 * M_PI * f2 * t), 0.0);

        yy_signal.push_back(currentSample);
        tt.push_back(t);
    }

    return yy_signal;
}

//------------------------------------------------------------------

std::tuple<std::vector<double>, std::vector<double>> computeTestSignal(int choice)
{

    std::vector<double> tt;
    std::vector<double> yy;

    double A1{1.0};
    double A2{0.2};
    double A3{0.6};

    double f1{2.0};
    double f2{50.0};
    double f4{20.0};
    double f5{80.0};

    int size = 1000;
    double dt = 1.0 / size;
    double yyi;

    if (choice == 1)
    {

        for (double t = 0.0; t <= 1.0; t = t + dt)
        {

            yyi = A1 * std::sin(2 * M_PI * f1 * t) + A2 * std::sin(2 * M_PI * f2 * t);
            tt.push_back(t);
            yy.push_back(yyi);
        }
    }

    if (choice == 2)
    {

        for (double t = 0.0; t <= 1.0; t = t + dt)
        {

            yyi = A1 * std::sin(2 * M_PI * f4 * t) + A3 * std::sin(2 * M_PI * f2 * t) + A1 * std::sin(2 * M_PI * f5 * t);

            tt.push_back(t);
            yy.push_back(yyi);
        }
    }

    return std::make_tuple(tt, yy);
}

//------------------------------------------------------------------

std::vector<std::complex<double>> lowPassFilter(std::tuple<std::vector<double>, std::vector<double>> testSignal)
{

    std::vector<double> tt = std::get<0>(testSignal);
    std::vector<double> yy = std::get<1>(testSignal);

    //implementation of low pass filter

    std::vector<double> yy_filtered(yy.size(), 0.0);

    double a1 = 0.96906992;
    double b0 = 0.01546504;
    double b1 = 0.01546504;

    for (int ii = 3; ii < yy.size(); ii++)
    {

        yy_filtered[ii] = a1 * yy_filtered[ii - 1] + b0 * yy[ii] + b1 * yy[ii - 1];
    }

    //output complex

    std::vector<std::complex<double>> outputCPX;

    for (auto &ii : yy_filtered)
    {

        auto outputCPXi = std::complex<double>(ii, 0.0);
        outputCPX.push_back(outputCPXi);
    }

    return outputCPX;
}


//------------------------------------------------------------------
void plot(std::vector<std::complex<double>> signalIN)
{

    std::vector<double> xx;
    std::vector<double> yy;

    int size = signalIN.size();
    double ii{0.0};
    for (auto &i : signalIN)
    {

        xx.push_back(ii);
        yy.push_back(std::abs(i));
        ii++;
    }
}

//------------------------------------------------------------------
void plot2(std::vector<std::complex<double>> signalIN)
{

    int N = signalIN.size();
    std::vector<double> xx;
    std::vector<double> yy;

    for (int n = 0; n < N; n++)
    {
        xx.push_back(n);
    }

    for (auto &ii : signalIN)
    {

        ii = ii / static_cast<double>(N);
        yy.push_back(std::abs(ii));
    }

}

//------------------------------------------------------------------
void plot3(std::vector<double> filteredSignalIN)
{

    int N = filteredSignalIN.size();
    std::vector<double> xx;
    std::vector<double> yy = filteredSignalIN;

    for (int n = 0; n < N; n++)
    {
        xx.push_back(n);
    }
    std::cout << " xx :: " << xx.size() << " :: " << yy.size() << std::endl;
    // for (auto &ii : signalIN){

    //     ii = ii/static_cast<double>(N);
    //     yy.push_back(std::abs(ii));
    // }

}

//------------------------------------------------------------------
std::vector<std::complex<double>> computeSignal()

{

    int N{1000};
    std::vector<std::complex<double>> signal;
    signal.reserve(N);

    std::vector<double> time;

    double sigK = 3.0;
    double sigPhase = M_PI / 4.0;
    int f1 = 2;
    int f2 = 50;
    //int f3 = 12;
    int A1 = 1;
    int A2 = 1;
    // int A3 = 20;

    for (int t = 0.0; t < N; ++t)
    {

        //auto currentSamle = std::complex<double>(std::cos((2 * M_PI / static_cast<double>(N)) * sigK * static_cast<double>(x) + sigPhase),  0.0);
        //auto currentSamle  =  std::complex<double> ((A1  *  std::cos( (2 * M_PI)/static_cast<double>(N) * f1 * t )  +
        //                                                              A2  *  std::cos( (2 * M_PI)/static_cast<double>(N) * f2 * t )  +
        //                                                              A3  *  std::cos ( (2 * M_PI)/static_cast<double>(N) * f3 * t )), 0.0);

        auto currentSamle = std::complex<double>(A1 * std::sin((2 * M_PI) / static_cast<double>(N) * f1 * t) + A2 * std::sin((2 * M_PI) / static_cast<double>(N) * f2 * t), 0.0);

        signal.push_back(currentSamle);
    }

    return signal;
}

//------------------------------------------------------------------

std::vector<std::complex<double>> computeDFT(std::vector<std::complex<double>> X)
{

    int N = X.size();
    int K = N;

    std::vector<double> dft;
    std::vector<int> freq;

    std::complex<double> intSum;

    std::vector<std::complex<double>> output;
    output.reserve(K);

    for (int k = 0; k < K; k++)
    {
        intSum = std::complex<double>(0.0, 0.0);

        for (int n = 0; n < N; n++)
        {

            double realPart = std::cos((2 * M_PI / N) * k * n);
            double imagPart = std::sin((2 * M_PI / N) * k * n);

            std::complex<double> w(realPart, -imagPart);

            intSum += X[n] * w;
        }

        output.push_back(intSum);
    }

    // take half due tp Nyquist

    std::vector<std::complex<double>> outputNyquist;

    // for(int ii = 0; ii<N/4; ii++){
    //      outputNyquist.push_back(output[ii]);
    // }
    // return outputNyquist;
    return output;
}

//------------------------------------------------------------------

std::vector<std::complex<double>> notchFilter(std::tuple<std::vector<double>, std::vector<double>> testSignal, int choice)
{

    std::vector<double> tt = std::get<0>(testSignal);
    std::vector<double> yy = std::get<1>(testSignal);

    //implementation of Notch filter

    //filter
    std::vector<double> yy_filtered(yy.size(), 0.0);

    if (choice == 1)
    {

        double a1 = 1.37624044;
        double a2 = -0.44587111;
        double b0 = 0.73401885;
        double b1 = -1.37624044;
        double b2 = 0.71185226;

        for (int ii = 3; ii < yy.size(); ii++)
        {

            yy_filtered[ii] = a1 * yy_filtered[ii - 1] + a2 * yy_filtered[ii - 2] + b0 * yy[ii] + b1 * yy[ii - 1] + b2 * yy[ii - 2];
        }
    }

    if (choice == 2)
    {

        double a1 = 1.37624044;
        double a2 = -0.44587111;
        double b0 = 0.73401885;
        double b1 = -1.37624044;
        double b2 = 0.71185226;

        for (int ii = 3; ii < yy.size(); ii++)
        {

            yy_filtered[ii] = a1 * yy_filtered[ii - 1] + a2 * yy_filtered[ii - 2] + b0 * yy[ii] + b1 * yy[ii - 1] + b2 * yy[ii - 2];
        }
    }

    //output complex

    std::vector<std::complex<double>> outputCPX;

    for (auto &ii : yy_filtered)
    {

        auto outputCPXi = std::complex<double>(ii, 0.0);
        outputCPX.push_back(outputCPXi);
    }
    return outputCPX;
}

//------------------------------------------------------------------

std::vector<std::complex<double>> computeDFT_IN(std::tuple<std::vector<double>, std::vector<double>> data)
{

    std::vector<double> X = std::get<1>(data);

    int N = X.size();
    int K = N;

    std::vector<std::complex<double>> dft;
    //std::vector<double> dft;
    //std::vector<int> freq;

    for (int k = 0; k <= K; k++)
    {
        std::complex<double> dft_k{0.0, 0.0};
        std::complex<double> dft_cpx{0.0, 0.0};

        for (int n = 0; n <= N - 1; n++)
        {

            dft_cpx = (std::cos((2 * M_PI / N) * k * n), -std::sin((2 * M_PI / N) * k * n));

            dft_k += X[n] * dft_cpx;
        }

        //dft.push_back(std::abs(dft_k));
        dft.push_back(dft_k);
        //freq.push_back(k);
    }
    return dft;
    //return std::make_tuple(freq, dft);
}

//------------------------------------------------------------------

int main()
{
    //ordinary LowPass Filter
    std::tuple<std::vector<double>, std::vector<double>> testSignal = computeTestSignal(1);

    std::vector<std::complex<double>> signalIN = computeDFT_IN(testSignal);

    plot2(signalIN);

    std::vector<std::complex<double>> filteredSignal = lowPassFilter(testSignal);

    std::vector<std::complex<double>> signalFilteredDFT = computeDFT(filteredSignal);


    plot2(signalFilteredDFT);

    
    // Notch filter

    
    std::tuple<std::vector<double>, std::vector<double>> testNotchSignal = computeTestSignal(1);

    std::vector<std::complex<double>> filteredNotchSignal = notchFilter(testNotchSignal, 1);

    std::vector<std::complex<double>> signalFilteredNotchDFT = computeDFT(filteredNotchSignal);

    plot2(signalFilteredNotchDFT);


    // Notch filter Signal 2

    std::tuple<std::vector<double>, std::vector<double>> testNotchSignal2 = computeTestSignal(2);

    std::vector<std::complex<double>> filteredNotchSignal2 = notchFilter(testNotchSignal2, 2);

    std::vector<std::complex<double>> signalFilteredNotchDFT2 = computeDFT(filteredNotchSignal2);

    plot2(signalFilteredNotchDFT2);


}