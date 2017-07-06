#include <LaterMethods.h>
vector<double> LaterMethods::getGaussian1D(double sigma, double precision)
{
    vector<double> kernel;
    int size = 1 + 2*ceil(precision*sigma);//ceil比sigma大的最小整数
    int center = size/2.0;//6

    double x,temp;
    double sum = 0.0;

    for (int i = 0; i < size; i++)
    {
        x = i-center;
        temp = exp(x*x/(-2.0*sigma*sigma))/(sigma*sqrt(2.0*M_PI));
        sum +=temp;
        kernel.push_back(temp);
    }

    for (int i = 0; i < size; i++)
    {
        kernel.at(i) /=sum;
//        printf("kernel11111111111 %f ",kernel.at(i));
    }

    return kernel;
}

void LaterMethods::conv1D(unsigned char *vec, const vector<double> &kernel, bool flag)
{
    int kernelSize = kernel.size();
    int kCenter = floor(kernelSize/2.0);

    int nn;

//    vector<double> result(vec.size(), 0.0);
    unsigned char *result=(unsigned char *)malloc(88000*sizeof(unsigned char));
    memset(result,0,88000*sizeof(unsigned char));
    if (!flag)
    {
        for(int j = 0; j < 88000; j++)
        {
            for (int n = 0; n < kernelSize; n++)
            {
                nn = kernelSize - 1 - n;
                int posx = j/400;//y
                int posy = j%400 + (n - kCenter);//x
                int w=posy*400+posx;
                if(posy >= 0 && posy < 220)
                {
                    result[j] += vec[w]*kernel.at(nn);
                }
            }
        }
    }
    else
    {
        for(int i = 0; i < 88000; i++)
        {

                for (int n = 0; n < kernelSize; n++)
                {
                    nn = kernelSize - 1 - n;
                    int posx = i%400 + (n - kCenter);//x
                    int posy = i/400;//y
                    int w=posx*400+posy;
                    if(posx >= 0 && posx < 400)
                    {
                        result[i] += vec[w]*kernel.at(nn);
                    }
                }

        }
    }
    memcpy(vec,result,88000*sizeof(unsigned char));
//    return *result;
}
void LaterMethods::MorphologyErode(unsigned char *vec)//腐蚀
{
    unsigned char *vec_copy=(unsigned char *)malloc(88000*sizeof(unsigned char));
    memcpy(vec_copy,vec,88000*sizeof(unsigned char));

    for (int i=0;i<88000;i++)
    {
       if (vec[i]==1)
       {
//           vec_copy[i]=0;
           int x=i%400;
           int y=i/400;
           int n[8] = { (x) + (y-1) * 400,(x-1) + (y) * 400, (x+1) + (y) * 400,(x) + (y+1) * 400,(x+1) + (y+1) * 400,(x-1) + (y+1) * 400,(x+1) + (y-1) * 400,(x-1) + (y-1) * 400};
           for (int ii=0;ii<4;ii++)
           {
               if ((vec[n[ii]]==0))
               {
                   vec_copy[i]=0;
//                   printf("aaa");
                   break;
               }
           }

       }
    }
    memcpy(vec,vec_copy,88000*sizeof(unsigned char));
}

void LaterMethods::MorphologyDilate(unsigned char *vec)//膨胀
{
    unsigned char *vec_copy=(unsigned char *)malloc(88000*sizeof(unsigned char));
    memcpy(vec_copy,vec,88000*sizeof(unsigned char));

    for (int i=0;i<88000;i++)
    {
       if (vec[i]==1)
            continue;
       if (vec[i]==0)
       {
//           vec_copy[i]=1;
           int x=i%400;
           int y=i/400;
           int n[8] = { (x) + (y-1) * 400,(x-1) + (y) * 400, (x+1) + (y) * 400,(x) + (y+1) * 400,(x+1) + (y+1) * 400,(x-1) + (y+1) * 400,(x+1) + (y-1) * 400,(x-1) + (y-1) * 400};
           for (int ii=0;ii<8;ii++)
           {
               if ((vec[n[ii]]==1))
               {
                   vec_copy[i]=1;
                   break;
               }
           }

       }
    }
    memcpy(vec,vec_copy,88000*sizeof(unsigned char));
}
