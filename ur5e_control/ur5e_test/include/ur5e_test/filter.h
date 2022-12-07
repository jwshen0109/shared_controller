#ifndef FILTER_H
#define FILTER_H

#define SUM_WIN_SIZE 10
#define DIMENSION_SIZE 6

class movingAveageFilter{

    public:
    
        double mFilter(double current, int id);

    private:

        double history[DIMENSION_SIZE][SUM_WIN_SIZE];

        int buff_init_x = 0;
        int buff_init_y = 0;
        int buff_init_z = 0;
        int buff_init_tx = 0;
        int buff_init_ty = 0;
        int buff_init_tz = 0;

        int index_x = 0;
        int index_y = 0;
        int index_z = 0;
        int index_tx = 0;
        int index_ty = 0;
        int index_tz = 0;

};

#endif