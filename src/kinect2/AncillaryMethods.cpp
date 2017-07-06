#include </home/abby/kinect2_ws/src/kinect2_driver/include/AncillaryMethods.h>



vector<double> AncillaryMethods::getGaussian1D(double sigma, double precision)
{
    vector<double> kernel;
    int size = 1 + 2*ceil(precision*sigma);//ceil比sigma大的最小整数
    kernel.setSize(size);//13
    int center = size/2.0;

    double x;
    double sum = 0.0;

    for (int i = 0; i < size; i++)
    {
        x = i-center;
        kernel(i) = exp(x*x/(-2.0*sigma*sigma))/(sigma*sqrt(2.0*M_PI));
        sum +=kernel(i);
    }

    for (int i = 0; i < size; i++)
    {
        kernel(i) /=sum;
    }

    return kernel;
}

vector<double> AncillaryMethods::conv1D(const Vector<double> &vec, const Vector<double> &kernel)
{
    int kernelSize = kernel.getSize();
    int kCenter = floor(kernelSize/2.0);

    int nn;

    vector<double> result(vec.getSize(), 0.0);

    double vec_first = vec(0), vec_last = vec(vec.getSize()-1);

    for(int j = 0; j < vec.getSize(); j++)
    {
        for (int n = 0; n < kernelSize; n++)
        {
            nn = kernelSize - 1 - n;
            int pos = j + (n - kCenter);
            if(pos<0)
            {
                result(j) += vec_first*kernel(nn);
            }
            else if(pos >= vec.getSize())
            {
                result(j) += vec_last*kernel(nn);
            }
            else//if(pos >= 0 && pos < vec.getSize())
            {
                result(j) += vec(pos)*kernel(nn);
            }
        }
    }

    return result;
}
