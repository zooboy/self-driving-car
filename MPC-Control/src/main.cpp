#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Here to define a function to transform the global coordinate to car's.
//displaycoord is a struct variable that defined in MPC.h
displaycoord get_displaypoint( vector<double> ref_x, vector<double> ref_y,double car_x,double car_y,double theta)
{
	displaycoord dispoint;

	for(size_t k = 0;k<ref_x.size();k++){
	double A = ref_x[k] - car_x;
	double B = ref_y[k] - car_y;
	dispoint.x.push_back(sin(theta)*B+cos(theta)*A);
	dispoint.y.push_back(cos(theta)*B-sin(theta)*A);
	}
	return dispoint;
}


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// this function is used for calculate the length of a fit polynomial
// it is an integral derivation from  Integral(sqrt(1+f'(x)))|(x0-xt)
// here I used a 2 order polynomial to approximate calculte the length,because 
// the coefficient for the 3rd order is very tiny,near 0, on the other hand 
// the  Integral(sqrt(1+f'(x))) derivation for 3 order polynomial is very
// complicated.
// By this way we can calculate a approximate length of polynomial for each time-step
// that can used for  estimate the duration of N*dt .
double clen (double x,double coeff0,double coeff1,double coeff2){

	double p0 = sqrt(1+pow(2*coeff2*x+coeff1,2));
	double p1 = 2*coeff2*x+coeff1;
	double p2 = p1/(4*coeff2)*p0;
	double p3 = p0+p1;
	double p4 = p0-p1;
	double pt = p2 + (1/8*coeff2)*log(p3/p4)+coeff0;

	return pt;

}



// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;


  
 // the code below is used for debug.
  double minx =2000;
  double minlen = 2000;
  double iter = 0.0;
  double totallen = 0.0;
  double avgv = 0.0;
  bool debug_flag = false;
  
 // h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {  
	

  h.onMessage([&mpc,&minx,&minlen,&iter,&totallen,&avgv,&debug_flag](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {

  // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
	
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  // Follow the instruction of this project,the steer direction is opposite to the quiz"mind the line".
		  double delta = j[1]["steering_angle"];
		  delta = -delta;
		  double acc = j[1]["throttle"];
		  //transform the speed from mph to m/s
		  v = v*0.44704;

		  //transform the reference trace global coordinate to car coordinate
		 displaycoord ref_trace;
		 ref_trace = get_displaypoint(ptsx,ptsy,px,py,psi);
		 Eigen::VectorXd ref_x = Eigen::VectorXd::Map(&ref_trace.x[0],ref_trace.x.size());
		 Eigen::VectorXd ref_y = Eigen::VectorXd::Map(&ref_trace.y[0],ref_trace.y.size());

		 // fit the 3 order polynomial based car's coordinate
		 Eigen::VectorXd coeffs = polyfit(ref_x,ref_y,3);




		 
		 //The Code BELOW is used to debug ,for estimating T= N*dt.
		 //I list them here for check and improve performance for future.
		 if(debug_flag==true){
			 iter +=1;
			 double dsxf = ref_trace.x[ref_trace.x.size()-1];
			 double dsyf = ref_trace.y[ref_trace.x.size()-1];
			 double dsx0 = ref_trace.x[0];
			 double dist = sqrt(pow(dsxf,2)+pow(dsyf,2));
			 double clen0 = clen(dsx0,coeffs[0],coeffs[1],coeffs[2]);
			 double clen1 = clen(dsxf,coeffs[0],coeffs[1],coeffs[2]);
			 double lenth = clen1 - clen0;
			 totallen += lenth;
			 avgv += v;
			 if(dist < minx){
				 minx = dist;
			 }
			 if(lenth<minlen)
			 {
				 minlen = lenth;
			 }
			 cout<<endl<<endl<<endl;
			 cout<<"dist:"<<dist<<",min:"<<minx<<endl;
			 cout<<" lenth:"<<lenth<<" ,minlen:"<<minlen<<" ,avglen:"<<totallen/iter<<endl;
			 cout<<"c0:"<<coeffs[0]<<",c1:"<<coeffs[1]<<",c2:"<<coeffs[2]<<",c3:"<<coeffs[3]<<endl;
			 cout<<"v:"<<v<<" ,AVG V:"<<avgv/iter<<endl;
			 cout<<endl<<endl<<endl;
		 }



		 double Lf = 2.67; 

		 //I think the most tricky part in this project is here:
		 //Since we transform global to car'coordinate, the car's position always at: x = 0,y =0 ,heading angle= 0;
		 //The latency,which means when receive sysytem state information ,the system has already run in this state for a while.
		 //So, when we consider the system current state ,we need to add the latency to sysytem, we need to calculate the system 
		 //state after latency and estimate the latency state in car's coordinate,in other words,we need to calculate the difference
		 //between the state in ideal(no latency) and the state in latency by car's coordinate.
		 //The psi_dest is ψdest, the acceleration of angular velocity ,psi_dest should be arctan(f'(xt)) ,as we know the derivative 
		 //of 3 order polynomial as:f'(xt) = (ax+bx*x+cx*x*x)' = a+2bx+3cx*x 
		 //consider the origin point x = 0 in car's coordinate,just as x(t) =0,bring x = 0 into f'(xt),psi_dest = arctan(a);
		 //Following the instruction of this project, the steer angle has an opposite direction compare to the formula used in lesson.
		 //So , the psi_dest = -atan(coeffs[1])
		  double psi_dest =- atan(coeffs[1]); 
		 
		  //Set the latency to 100ms,I tried some values of latancy, 100ms  seems perform better.  
		  double latency = 0.10;  // 100 ms
		  
		  //The 100ms duration is very short, we can think the car drives in a stright line,that means car drives 
		  //along x-axis in cars'coordinate,  
		  //So the x_lt = v*latency; y_lt = 0;
		  double x_lt = v*latency;
          double y_lt = 0;
		
		  // As heading angle = 0 in ideal state,follow the formula ψt+1=ψt+vt/Lf*δ*dt, the psi after latency as "psi_lt". 
		  // the delta is angular velocity represented by negative "steer angle" that received from sensor. 
		  double psi_lt = v/Lf* delta * latency;

		  //follow the formula for Cte(t+1) , as yt = 0,the Cte(t) = polyeval(coeffs, 0), the formula can express like this:
		  // The orientation of variable confused me a lot.


		  double cte_lt = polyeval(coeffs, 0) + v * sin(psi_dest)*latency;

		  //As epsi(t) = psi_dest,the epsi(t+1) = epsi(t) + v/Lf* delta * dt
		  double epsi_lt = psi_dest+ v/Lf* delta * latency  ;
          double v_lt = v + acc * latency;

		
		  Eigen::VectorXd state(6);
		  state << x_lt, y_lt, psi_lt, v_lt, cte_lt, epsi_lt;


		  std::vector<double> x_vals = {state[0]};
		  std::vector<double> y_vals = {state[1]};
          std::vector<double> psi_vals = {state[2]};
		  std::vector<double> v_vals = {state[3]};
		  std::vector<double> cte_vals = {state[4]};
		  std::vector<double> epsi_vals = {state[5]};
		  std::vector<double> delta_vals = {};
		  std::vector<double> a_vals = {};

		  // get_result is a struct variable that defined in MPC.h, the variable is used to get the optimization results.
		  
		  get_result vars = mpc.Solve(state, coeffs);
		  
		
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

		  //The steer value that send back from the car in the instruction in "Websocket Data" was :
		  //"The current steering angle in radians"; And I don't understand why need to divide by deg2rad(25);
		  // I've noticed some students said the simulator worked in [-degree 25, degree 25],and the 
		  // "steering_angle" scaled [-degree 25, degree 25] to [-1,1], in that case , should I need to 
		  // convert the send back  "steering_angle" value  to [-deg2rad(25), deg2rad(25)] first ?
		  //Whether I missed something in the lesson ?

		  //I don't convert this value ,but the car drives smooth.
		  double steer_value= vars.steer;
		  double throttle_value=  vars.throttle;
		 // cout<< "steer value:"<< steer_value<<",throttle: "<<throttle_value<<", psi:"<<psi<< endl;
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
		  //get the display points that predicted by MPC model.
		  mpc_x_vals = vars.predlist_x;
		  mpc_y_vals = vars.predlist_y;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  // Display 30 points trajectory of refercence polynomial with 1 m step .
		for(int i = 1 ;i<=30;i++){

			next_x_vals.push_back(i);
			next_y_vals.push_back(polyeval(coeffs,i));

		}
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
         // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
