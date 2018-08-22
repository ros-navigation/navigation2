#ifndef ENV_MODEL_OCC_GRID_LAYER_H
#define ENV_MODEL_OCC_GRID_LAYER_H

#include <string>
#include <vector>



struct MyLayer {


    // Layer Meta-Info

    virtual ~MyLayer() {}

    // Get Layer Info
    virtual std::string get_name() = 0;
    virtual std::string get_type() = 0;
    virtual int get_size()= 0;
    virtual int get_sizex()= 0;
    virtual int get_sizey()= 0;
    virtual double get_originx() = 0;
    virtual double get_originy() = 0;
    virtual double get_res() = 0;
    
    // Set Layer Info
    virtual void set_name(std::string name) = 0;
    virtual void set_type(std::string name) = 0; //probably not needed


    // Get Data (as float) by index or pointer  
    virtual float get_value(int index) = 0;
    virtual float* get_data()= 0;
    virtual float* get_data(std::vector<int> index) = 0;

    // Set Data by single index or vector
    virtual void set_data(int index, double value) =0;
    virtual void set_data(int index, signed char value)=0;
    virtual void set_data(int index, unsigned char value)=0;
    virtual void set_data(int index, float value) =0;

    virtual void set_data(std::vector<int> index, double* values) = 0;
    virtual void set_data(std::vector<int> index, signed char* values) = 0;
    virtual void set_data(std::vector<int> index, unsigned char* values) = 0;
    virtual void set_data(std::vector<int> index, float* values) = 0;

};



template <class T>
struct MyData : MyLayer{
    
    
    double resolution_;
    int size_x_;
    int size_y_;
    double origin_x_;
    double origin_y_; 
    int size;

    std::string name_;
    std::string data_type;
    T data_;
    

    MyData(std::string name,T data, int size_x, int size_y) :data_(data), name_(name), size_x_(size_x_), size_y_(size_y){size = size_x*size_y;}
    
    MyData(std::string name,T data, int size_x, int size_y, double resolution, double origin_x, double origin_y) :
        data_(data), name_(name), size_x_(size_x_), size_y_(size_y), resolution_(resolution),origin_x_(origin_x), origin_y_(origin_y)
        {size = size_x*size_y;}
     
    // Get Layer Info 

    std::string get_name() {return name_;}
    std::string get_type()  {return data_type;}
    int get_size(){return size;}
    int get_sizex(){return size_x_;}
    int get_sizey(){return size_y_;}
    double get_originx() {return origin_x_;}
    double get_originy() {return origin_y_;}
    double get_res(){return resolution_;}


    // Set Layer Info 

    void set_name(std::string name)  {name_ = name;}
    void set_type(std::string type)  {data_type = type;} 


    // Get Data

    float get_value(int index)  {return (float) data_[index];} 

    float* get_data()
    {
        float * d;
        d = (float*) malloc(size*sizeof(float));
         
        for (int i = 0; i < size; i++)
        {
            d[i] = (float) data_[i];
        }
        return d;
    
    }

    float* get_data(std::vector<int> index)
    {
        float * d;
        int isize = index.size();
        d = (float*) malloc(isize*sizeof(float));
        
        for (int i = 0; i < isize; i++)
        {
            d[i] = (float) data_[index[i]];
        }
        return d;
    
    }
    // Set Data

    void set_data(int index, double value){data_[index] = value;}
    void set_data(int index, signed char value){data_[index] = value;};   
    void set_data(int index, unsigned char value){data_[index] = value;};   
    void set_data(int index, float value){data_[index] = value;};   
   
    void set_data(std::vector<int> index, double* values)
    {
        int isize = index.size();        
        for (int i = 0; i < isize; i++)
        {
            data_[index[i]] = values[i];;
        }
        
    }
    void set_data(std::vector<int> index, signed char* values)
    {
        int isize = index.size();        
        for (int i = 0; i < isize; i++)
        {
            data_[index[i]] = values[i];;
        }        
    }
    void set_data(std::vector<int> index, unsigned char* values)
    {
        int isize = index.size();        
        for (int i = 0; i < isize; i++)
        {
            data_[index[i]] = values[i];;
        }        
    }
        
    void set_data(std::vector<int> index, float* values)
    {
        int isize = index.size();        
        for (int i = 0; i < isize; i++)
        {
            data_[index[i]] = values[i];;
        }

    }

    
};

/*
template <class T>
struct layer_M
{   
    std::string name;
    std::string data_type;   
    T data;
};



struct layer_t
{   
    std::string name;
    std::string data_type;   

    union {
        signed char * data;  
        double * ddata;  
    };
};
*/

#endif