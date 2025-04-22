/**
 * @brief 		a library for storage datas using embedded flash
 * @detail
 * @author 	    Haoqi Liu
 * @date        25-4-22
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		history
                V1.0.0 on 25-4-22
 * */

#ifndef STORAGE_H
#define STORAGE_H

class Storage {
public:
    virtual ~Storage() = default;
    // user should define constructor self, just to assign the member variables. it should decouple from the hardware

    virtual void bind() = 0;

    virtual void write() = 0;

    virtual void read() = 0;
};

#endif //STORAGE_H
