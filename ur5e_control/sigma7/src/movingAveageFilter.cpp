#include "sigma7/movingAveageFilter.h"

double movingAveageFilter::mFilter(double current, int id)
{
    double sum = 0;

    switch(id){
        case 0:
            if(buff_init_x == 0){

                history[0][index_x] = current;
                index_x++;

                if(index_x >= (SUM_WIN_SIZE-2)){
                    buff_init_x = 1;
                }
                return 0;

            }else{

                history[0][index_x] = current;
                index_x++;

                if(index_x >= SUM_WIN_SIZE)
                {
                    index_x = 0;
                }

                for(int i=0; i < SUM_WIN_SIZE; i++)
                {
                    sum+=history[0][i];
                }

                return sum/SUM_WIN_SIZE;
            }
        case 1:
            if(buff_init_y == 0){
                history[1][index_y] = current;
                index_y++;

                if(index_y >= (SUM_WIN_SIZE-2)){
                    buff_init_y = 1;
                }

                return 0;

            }else{
                history[1][index_y] = current;
                index_y++;

                if(index_y >= SUM_WIN_SIZE)
                {
                    index_y = 0;
                }

                for(int i=0; i < SUM_WIN_SIZE; i++)
                {
                    sum+=history[1][i];
                }

                return sum/SUM_WIN_SIZE;
            }
        case 2:
            if(buff_init_z == 0){
                history[2][index_z] = current;
                index_z++;

                if(index_z >= (SUM_WIN_SIZE-2)){
                    buff_init_z = 1;
                }

                return 0;
            }else{
                history[2][index_z] = current;
                index_z++;

                if(index_z >= SUM_WIN_SIZE)
                {
                    index_z = 0;
                }

                for(int i=0; i < SUM_WIN_SIZE; i++)
                {
                    sum+=history[2][i];
                }

                return sum/SUM_WIN_SIZE;
            }
        case 3:
            if(buff_init_tx == 0){
                history[3][index_tx] = current;
                index_tx++;

                if(index_tx >= (SUM_WIN_SIZE-2)){
                    buff_init_tx = 1;
                }

                return 0;
            }else{
                history[3][index_tx] = current;
                index_tx++;

                if(index_tx >= SUM_WIN_SIZE)
                {
                    index_tx = 0;
                }

                for(int i=0; i < SUM_WIN_SIZE; i++)
                {
                    sum += history[3][i];
                }

                return sum/SUM_WIN_SIZE;
            }
        case 4:
            if(buff_init_ty == 0){

                history[4][index_ty] = current;
                index_ty++;

                if(index_ty >= (SUM_WIN_SIZE-2)){
                    buff_init_ty = 1;
                }

                return 0;
            }else{

                history[4][index_ty] = current;
                index_ty++;
                
                if(index_ty >= SUM_WIN_SIZE)
                {
                    index_ty = 0;
                }

                for(int i=0; i < SUM_WIN_SIZE; i++)
                {
                    sum += history[4][i];
                }

                return sum/SUM_WIN_SIZE;
            }
        case 5:
            if(buff_init_tz == 0){

                history[5][index_tz] = current;
                index_tz++;

                if(index_tz >= (SUM_WIN_SIZE-2)){
                    buff_init_tz = 1;
                }

                return 0;

            }else{
                history[5][index_tz] = current;
                index_tz++;

                if(index_tz >= SUM_WIN_SIZE)
                {
                    index_tz = 0;
                }

                for(int i=0; i < SUM_WIN_SIZE; i++)
                {
                    sum += history[5][i];
                }

                return sum/SUM_WIN_SIZE;
            }

    }
 
}