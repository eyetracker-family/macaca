#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

/*
Copyright 2015 Universität Tübingen

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT 
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN 
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER 
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
  Version 1.0, 17.12.2015, Copyright University of Tübingen.

  The Code is created based on the method from the paper:
  "ElSe: Ellipse Selection for Robust Pupil Detection in Real-World Environments", W. Fuhl, T. C. Santini, T. C. Kübler, E. Kasneci
  ETRA 2016 : Eye Tracking Research and Application 2016
 
  The code and the algorithm are for non-comercial use only.

*/

namespace ELSE{


static void filter_edges(cv::Mat *edge, int start_xx, int end_xx, int start_yy, int end_yy){



		int start_x=start_xx+5;
		int end_x=end_xx-5;
		int start_y=start_yy+5;
		int end_y=end_yy-5;


		if(start_x<5) start_x=5;
		if(end_x>edge->cols-5) end_x=edge->cols-5;
		if(start_y<5) start_y=5;
		if(end_y>edge->rows-5) end_y=edge->rows-5;




		for(int j=start_y; j<end_y; j++)
		for(int i=start_x; i<end_x; i++){
			int box[9];

			box[4]=(int)edge->data[(edge->cols*(j))+(i)];

			if(box[4]){
				box[1]=(int)edge->data[(edge->cols*(j-1))+(i)];
				box[3]=(int)edge->data[(edge->cols*(j))+(i-1)];
				box[5]=(int)edge->data[(edge->cols*(j))+(i+1)];
				box[7]=(int)edge->data[(edge->cols*(j+1))+(i)];


				if((box[5] && box[7])) edge->data[(edge->cols*(j))+(i)]=0;
				if((box[5] && box[1])) edge->data[(edge->cols*(j))+(i)]=0;
				if((box[3] && box[7])) edge->data[(edge->cols*(j))+(i)]=0;
				if((box[3] && box[1])) edge->data[(edge->cols*(j))+(i)]=0;

			}
		}



		//too many neigbours
		for(int j=start_y; j<end_y; j++)
		for(int i=start_x; i<end_x; i++){
			int neig=0;

			for(int k1=-1;k1<2;k1++)
				for(int k2=-1;k2<2;k2++){

					if(edge->data[(edge->cols*(j+k1))+(i+k2)]>0)
						neig++;
				}

			if(neig>3)
				edge->data[(edge->cols*(j))+(i)]=0;

		}








}


}
