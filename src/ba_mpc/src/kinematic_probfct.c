/* This file is part of GRAMPC - (https://github.com/grampc/grampc)
 *
 * GRAMPC -- A software framework for embedded nonlinear model predictive
 * control using a gradient-based augmented Lagrangian approach
 *
 * Copyright 2014-2025 by Knut Graichen, Andreas Voelz, Thore Wietzke,
 * Tobias Englert (<v2.3), Felix Mesmer (<v2.3), Soenke Rhein (<v2.3),
 * Bartosz Kaepernick (<v2.0), Tilman Utz (<v2.0).
 *
 * GRAMPC is distributed under the BSD-3-Clause license, see LICENSE.txt
 *
 *
 *
 *
 *
 *
 *
 * This probfct file provides an interface to GRAMPC. The most general
 * formulation of the optimal control problem (OCP) that can be solved
 * by GRAMPC has the following structure
 *                                           _T
 *	                                    /
 *      min    J(u,p,T;x0) = V(T,x(T),p) + / l(t,x(t),u(t),p) dt
 *   u(.),p,T                            _/
 *                                      0
 *              .
 *      s.t.   Mx(t) = f(t0+t,x(t),u(t),p), x(0) = x0
 *             g(t,x(t),u(t),p)  = 0,   gT(T,x(T),p)  = 0
 *             h(t,x(t),u(t),p) <= 0,   hT(T,x(t),p) <= 0
 *             u_min <= u(t) <= u_max
 *             p_min <=  p   <= p_max
 *             T_min <=  T   <= T_max
 *
 *
 */

#include "probfct.h"
#include "math.h"
#include "my_cpp_py_pkg/userparam.h"

#if USE_typeRNum == USE_FLOAT
#define SIN(a)		sinf(a)
#define COS(a)		cosf(a)
#define TAN(a)      tanf(a)
#else
#define SIN(a)		sin(a)
#define COS(a)		cos(a)
#define TAN(a)      tan(a)
#endif

/* square macro */
#define POW2(a) ((a)*(a))

#define PI 3.14159265358979323846

/* //define point struct 
typedef struct {
    double x;
    double y;
    double yaw; //not needed i think
} my_point; */

//define point struct 
typedef struct {
    double x;
    double y;
    double yaw; //not needed i think
    double proj;
} my_point;


//functions i might need 

/*  get nearest point in Waypoint or border list, relative to car pos
    needs 3 values per point in list  */
typeInt getNearestIndex(double current_x, double current_y, const double* ref, int ref_length) {
    //warum 0 und nicht -1? no reason
    int nearest_idx = 0;
    double min_dist = 10000000.0; // Note: Hardcoded

    for (int i = 0; i < ref_length; i++) {
        double x = ref[3 * i];
        double y = ref[3 * i + 1];
        double d = sqrt((x - current_x) * (x - current_x) + (y - current_y) * (y - current_y));
        if (d < min_dist) {
            min_dist = d;
            nearest_idx = i;
        }
    }

    return nearest_idx;
}

//sorgt dafür das der yaw error immer zwischen pi und -pi liegt und nicht bei 2 pi wenn die abweichung 
typeRNum wrapToPi(const typeRNum number){
    typeRNum wrappedNumber = fmod(number + PI, 2.0*PI);
    if (wrappedNumber < 0) wrappedNumber += 2.0 * PI;
    wrappedNumber -= PI;

    return wrappedNumber;
}

// TODO add point projection fct here?
/* //einfache orthogonal proj für je drei Werte pro punkt
//ref: list of line points
//ref_length: length of ref
//x: point to proj 
my_point calculate_projected_ref_point(double* ref, int ref_length, my_point x){

    int nearest_idx = getNearestIndex(x.x, x.y, ref, ref_length);
    int next_idx = (nearest_idx + 1)%ref_length;

    // Coords of nearest point
    double nearest_x = ref[3*nearest_idx];
    double nearest_y = ref[3*nearest_idx + 1];

    // Coords of next point
    double next_x = ref[3*next_idx];
    double next_y = ref[3*next_idx + 1];

    // Vector entries from nearest to current position
    double nearest_to_current_x = x.x - nearest_x;
    double nearest_to_current_y = x.y - nearest_y;

    // Normalized vector entries from nearest to next point
    double nearest_to_next_x = next_x - nearest_x;
    double nearest_to_next_y = next_y - nearest_y;

    double nearest_to_next_length = sqrt(POW2(nearest_to_next_x) + POW2(nearest_to_next_y));
    nearest_to_next_x /= nearest_to_next_length;
    nearest_to_next_y /= nearest_to_next_length;

    // Projection of vector nearest_to_current onto nearest_to_next, resulting in the reference x and y
    double proj = (nearest_to_current_x * nearest_to_next_x) + (nearest_to_current_y * nearest_to_next_y); // dot(ntc, ntn)

    double x_ref = nearest_x + proj * nearest_to_next_x;
    double y_ref = nearest_y + proj * nearest_to_next_y;
    double yaw_ref = ref[3*nearest_idx +2];

    // Result structure
    my_point ref_point;

    ref_point.x = x_ref;
    ref_point.y = y_ref;
    ref_point.yaw = yaw_ref;

    return ref_point;
} */


/* calculates orthogonal projection of x on vector ab 
für je 2 Werte pro punkt
a: starting point vector
b: end point vector
x :point to be projected
returns: global projected point and proj relative to vector
---------------------------------------------------------------*/

my_point orhtogonal_proj(my_point a, my_point b, my_point x){

    // Vector entries from nearest to current position
    double a_to_current_x = x.x - a.x;
    double a_to_current_y = x.y - a.y;

    // Normalized vector entries from nearest to next point
    double a_to_b_x = b.x - a.x;
    double a_to_b_y = b.y - a.y;

    double a_to_b_length = sqrt(POW2(a_to_b_x) + POW2(a_to_b_y));
    a_to_b_x /= a_to_b_length;
    a_to_b_y /= a_to_b_length;

    // Projection of vector nearest_to_current onto nearest_to_next, resulting in the reference x and y
    //proj ist eine relative Länge von a_to_b
    //Falls proj <0: orthogonale Projektion vor a; falls proj <1: hinter b 
    double proj = (a_to_current_x * a_to_b_x) + (a_to_current_y * a_to_b_y); // dot(ntc, ntn)
    
    double x_ref = a.x + proj * a_to_b_x;
    double y_ref = a.y + proj * a_to_b_y;
    double yaw_ref = a.yaw;

    // Result structure
    my_point ref_point;

    ref_point.x = x_ref;
    ref_point.y = y_ref;
    ref_point.yaw = yaw_ref;
    ref_point.proj = proj;

    return ref_point;
}
//vielleicht auch abstandsberechnung? transformation?

//möglich als bool? x<0 or x>0 
//helps decide wich border to take as boundary
my_point transform_car_pos(my_point proj_point, my_point car_pos){
    
    double yaw_ref = proj_point.yaw;

    double xt = car_pos.x - proj_point.x;
    double yt = car_pos.y - proj_point.y;

    double xr = xt * cos(yaw_ref) - yt * sin(yaw_ref);
    double yr = xt *sin(yaw_ref) + yt * cos(yaw_ref);
    
    my_point transf_car_pos;
    transf_car_pos.x = xr;
    transf_car_pos.y = yr;
    //yaw not needed, can be left empty?

    return transf_car_pos;    
}

typeRNum euclidian_distance(my_point point1, my_point point2){
    double dist_sqrd = POW2(point1.x - point2.x) + POW2(point1.y - point2.y);
    return sqrt(dist_sqrd);
}

my_point find_proj_border_point(double* border, int border_len, my_point car_pos){
    //find border point to compare too, might not be exact enough
    int nearest_border_idx = getNearestIndex(car_pos.x, car_pos.y, border, border_len);
    int prev_border_idx = (nearest_border_idx-1) %border_len;
    int next_border_idx = (nearest_border_idx+1) %border_len;

    my_point prev_border_point;
    prev_border_point.x = border[3*prev_border_idx];
    prev_border_point.y = border[3*prev_border_idx +1];

    my_point nearest_border_point;
    nearest_border_point.x = border[3*nearest_border_idx];
    nearest_border_point.y = border[3*nearest_border_idx +1];

    my_point next_border_point;
    next_border_point.x = border[3*next_border_idx];
    next_border_point.y = border[3*next_border_idx +1];
    my_point proj_border_point;

    my_point car_on_prev_to_near = orhtogonal_proj(prev_border_point, nearest_border_point, car_pos);
    my_point car_on_near_to_next = orhtogonal_proj(nearest_border_point, next_border_point, car_pos);

    if((car_on_prev_to_near.proj < 1.0) &&  (car_on_near_to_next.proj <= 0.0) ){
        //use prev to near as reference
        //printf("case 1: proj on prev =  %f and proj on next= %f \n", car_on_prev_to_near.proj, car_on_near_to_next.proj);
        proj_border_point.x = car_on_prev_to_near.x;
        proj_border_point.y = car_on_prev_to_near.y;
        
    }
    else if((car_on_near_to_next.proj > 0.0) &&  (car_on_near_to_next.proj < 1.0)){
        //use near to next as ref
        //printf("case 2: proj on prev =  %f and proj on next= %f \n", car_on_prev_to_near.proj, car_on_near_to_next.proj);
        proj_border_point.x = car_on_near_to_next.x;
        proj_border_point.y = car_on_near_to_next.y;
    } 
    else if((car_on_prev_to_near.proj >0.0) && (car_on_prev_to_near.proj < 1.0 )&& (car_on_near_to_next.proj >=1.0)){
        //use prev to near
        //printf("case 3: proj on prev =  %f and proj on next= %f \n", car_on_prev_to_near.proj, car_on_near_to_next.proj);
        proj_border_point.x = car_on_prev_to_near.x;
        proj_border_point.y = car_on_prev_to_near.y;
    }
    else if((car_on_prev_to_near.proj >= 1.0) && (car_on_near_to_next.proj <= 0.0)){
        //use z_achse des kreuzproduktes
        //printf("case 4: proj on prev =  %f and proj on next= %f \n", car_on_prev_to_near.proj, car_on_near_to_next.proj);
        double car_to_next_proj_x = car_on_near_to_next.x - car_pos.x;
        double car_to_next_proj_y = car_on_near_to_next.y - car_pos.y;
        double near_to_next_x = next_border_point.x - nearest_border_point.x;
        double near_to_next_y = next_border_point.y - nearest_border_point.y;
        //(near_to_next x car_to_proj_point).z 
        double cross_z =  near_to_next_x * car_to_next_proj_y - near_to_next_y *car_to_next_proj_x;
        //printf("cross z=  %f  \n", cross_z);
        //notsure ob die richtung stimmt 
        if (cross_z < 0.0){
            proj_border_point.x = car_on_prev_to_near.x;
            proj_border_point.y = car_on_prev_to_near.y;
        }
        //(near_to_next x car_to_proj_point).z >0 => use next
        else{
            proj_border_point.x = car_on_near_to_next.x;
            proj_border_point.y = car_on_near_to_next.y;
        }
    }
    else if( (car_on_prev_to_near.proj >=1.0) && (car_on_near_to_next.proj >=1.0)){
        //use next
        //printf("case 5: proj on prev =  %f and proj on next= %f \n", car_on_prev_to_near.proj, car_on_near_to_next.proj);
        proj_border_point.x = car_on_near_to_next.x;
        proj_border_point.y = car_on_near_to_next.y;
    }
    else if((car_on_prev_to_near.proj <=0.0) && (car_on_near_to_next.proj>=1.0) ){
        //should not happen but if just use next 
        //printf("case 6: proj on prev =  %f and proj on next= %f \n", car_on_prev_to_near.proj, car_on_near_to_next.proj);
        proj_border_point.x = car_on_near_to_next.x;
        proj_border_point.y = car_on_near_to_next.y;
    }
    else{
        printf("new case not yet handled");
    }

    
    return proj_border_point;
}



/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng), 
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    *Nx = 4;    //Number of states [x, y, yaw, v]
    *Nu = 2;    //Number of controls [steering, acceleration]
    *Np = 0;    //Number of paramters
    *Nh = 3;    //Number of inequalities (eigenltich 3)
    *Ng = 0;    //Number of equalities
    *NgT = 0;
    *NhT = 0;

}

//was ist out?
//why no typeGrampcparam -> v.2.2
/** System function f(t,x,u,p,param,userparam) 
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*) userparam;
    double L = param->wheelbase; //abstand vorder radachse hintere Radachse

    out[0] = x[3] * COS(x[2]);            //dx/dt =v*cos(theta)
    out[1] = x[3] * SIN(x[2]);            // dy/dt = v * sin(theta)
    out[2] = (x[3] / L) * TAN(u[0]);      // dtheta/dt = v / L * tan(steering)
    out[3] = u[1];                        // dv/dt = a
}

/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    // vec zur opimierung der berechnung i guess
    UserParam* param = (UserParam*)userparam;
    double L = param->wheelbase;

    out[0] = 0.0; //ableitung nach X[0]
    out[1] = 0.0;
    out[2] = -x[3] * SIN(x[2]) * vec[0] + x[3] * COS(x[2]) * vec[1];                //-v*sin(theta)*vec[0] + v*cos(theta)*vec[1]
    out[3] = vec[0] * COS(x[2]) + vec[1] * SIN(x[2]) + vec[2] * TAN(u[0]) / L;      //vec[0]*cos(theta) + vec[1]*sin(theta) + vec[2]*tan(steering_angle)/L

    //t wird nicht verwendet?
    //printf("dfdx: yaw=%.3f, v=%3.f, vec0=%.3f, vec1=%.3f, vec2=%.3f \n", x[2], x[3], vec[0], vec[1], vec[2]);

}
/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;
    double L = param->wheelbase;

    out[0] = (x[3] / L) * (1.0 / POW2(COS(u[0]))) * vec[2]; //steering = (v/L) *(1/COS(steering)^2) *vec[2]
    out[1] = vec[3];                                        //a = 1*vec[3]

}
/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}


/** Integral cost l(t,x(t),u(t),p,param,userparam) 
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*) userparam;
    double* ref_traj = param->ref_traj;
    int ref_length = param ->ref_length; //was ist ref_length? vielleicht die waypoint listen länge

    int nearest_idx = getNearestIndex(x[0], x[1], ref_traj, ref_length);
    int next_idx = (nearest_idx + 1)%ref_length; //warum modulo ref_length? falls liste zuende 

    //printf("nearest idx: %d, next idx: %d t: %f \n", nearest_idx, next_idx, t);

    // Coords of nearest point
    double nearest_x = ref_traj[3*nearest_idx];
    double nearest_y = ref_traj[3*nearest_idx + 1]; 

    // Coords of next point
    double next_x = ref_traj[3*next_idx];
    double next_y = ref_traj[3*next_idx + 1];

    // Vector entries from nearest to current position
    double nearest_to_current_x = x[0] - nearest_x;
    double nearest_to_current_y = x[1] - nearest_y;

    // Normalized vector entries from nearest to next point
    double nearest_to_next_x = next_x - nearest_x;
    double nearest_to_next_y = next_y - nearest_y;
    double nearest_to_next_length = sqrt(POW2(nearest_to_next_x) + POW2(nearest_to_next_y)); //normalisierter Vector

    nearest_to_next_x /= nearest_to_next_length;
    nearest_to_next_y /= nearest_to_next_length;

    // Projection of vector nearest_to_current onto nearest_to_next, resulting in the reference x and y
    double proj = (nearest_to_current_x * nearest_to_next_x) + (nearest_to_current_y * nearest_to_next_y); // dot(ntc, ntn)

    double x_ref = nearest_x + proj * nearest_to_next_x; 
    double y_ref = nearest_y + proj * nearest_to_next_y;

    // The yaw reference can be taken from the provided trajectory at nearest idx
    double yaw_ref = ref_traj[3*nearest_idx + 2];

    //printf("Current x: %f, y: %f, v: %f, steering: %f, acc: %f \n", x[0], x[1], x[3], u[0], u[1]);

    //cost function
    out[0] =  param->Q_pos   * (POW2(x[0] - x_ref) + POW2(x[1] - y_ref)) //cte 
            + param->Q_theta * POW2(wrapToPi(x[2] - yaw_ref)) //heading error
            + param->Q_vel   * POW2(x[3] - param->max_velocity) //velocity error
            + param->R_steer * POW2(u[0]) //steering error to avoid jittering
            + param->R_accel * POW2(u[1]); // R_accel may be 0  
            // No term for acceleration yet todo 
            
    //printf("x_ref: %.3f, y_ref: %.3f, yaw_ref: %.3f , yaw: %.3f \n",x_ref,y_ref,yaw_ref, x[2]);


}

/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;

    double* ref_traj = param->ref_traj;
    int ref_length = param->ref_length;

    int nearest_idx = getNearestIndex(x[0], x[1], ref_traj, ref_length);
    int next_idx = (nearest_idx + 1)%ref_length;

    // Coords of nearest point
    double nearest_x = ref_traj[3*nearest_idx];
    double nearest_y = ref_traj[3*nearest_idx + 1];

    // Coords of next point
    double next_x = ref_traj[3*next_idx];
    double next_y = ref_traj[3*next_idx + 1];

    // Vector entries from nearest to current position
    double nearest_to_current_x = x[0] - nearest_x;
    double nearest_to_current_y = x[1] - nearest_y;

    // Normalized vector entries from nearest to next point
    double nearest_to_next_x = next_x - nearest_x;
    double nearest_to_next_y = next_y - nearest_y;

    double nearest_to_next_length = sqrt(POW2(nearest_to_next_x) + POW2(nearest_to_next_y));
    //printf("length: %f \n", nearest_to_next_length);
    nearest_to_next_x /= nearest_to_next_length;
    nearest_to_next_y /= nearest_to_next_length;

    // Projection of vector nearest_to_current onto nearest_to_next, resulting in the reference x and y
    double proj = (nearest_to_current_x * nearest_to_next_x) + (nearest_to_current_y * nearest_to_next_y); // dot(ntc, ntn)

    double x_ref = nearest_x + proj * nearest_to_next_x;
    double y_ref = nearest_y + proj * nearest_to_next_y;

    // The yaw reference can be taken from the provided trajectory at nearest idx
    double yaw_ref = ref_traj[3*nearest_idx + 2];

    //ableitung der einzelnen cost func aspekte? warum einzeln? weil ableitungen nach einzelnen states
    out[0] = param->Q_pos   * 2.0 * (x[0] - x_ref);
    out[1] = param->Q_pos   * 2.0 * (x[1] - y_ref);
    out[2] = param->Q_theta * 2.0 * (wrapToPi(x[2] - yaw_ref));
    out[3] = param->Q_vel   * 2.0 * (x[3] - param->max_velocity); 

}
/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;
    
    //ableitung nach output u[]
    out[0] = param->R_steer * 2.0 * u[0];
    out[1] = 0.0; // No term for the acceleration yet 

}

/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
}


/** Terminal cost V(T,x(T),p,param,userparam) 
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}
/** Gradient dV/dx **/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}
/** Gradient dV/dp **/
void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}
/** Gradient dV/dT **/
void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}


/** Equality constraints g(t,x(t),u(t),p,param,userparam) = 0 
    --------------------------------------------------- **/
void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Inequality constraints h(t,x(t),u(t),p,param,userparam) <= 0 
 * gesammte h fct soll kleiner als Null bleiben -> out [] = term kleiner als 0
    ------------------------------------------------------ **/
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    //inquality restraints
    UserParam* param = (UserParam*)userparam;

    double* center_traj = param->center_traj;
    int center_traj_len = param->center_traj_len;
    
    double car_width = param->width;

    double* outer_border = param->outer_border;
    int outer_border_len = param->outer_border_len;

    double* inner_border = param->inner_border;
    int inner_border_len = param->inner_border_len;

    double* border;
    int border_len;


    my_point car_pos;
    car_pos.x = x[0];
    car_pos.y = x[1]; 
    //yaw can be left empty 

    //2 verschiedene restraints für v min und v max 
    // todo stehen und rückwärtsfahren erlauben  auto fährt trotzdem rückwärts
    out[0] = x[3] - 1.2* param->max_velocity;    // v <= 1.2 * v_max //um wiedersprüche mit optimaler geschwindigkeit zu vermeiden   
    out[1] = -x[3];                         // 0 <= v

    //border constraint 
    //project car pos onto centerline

    int nearest_center_idx = getNearestIndex(car_pos.x, car_pos.y,  center_traj, center_traj_len);
    // Coords of nearest point
    my_point nearest_center_point;
    nearest_center_point.x = center_traj[3*nearest_center_idx];
    nearest_center_point.y = center_traj[3*nearest_center_idx + 1];

    my_point next_center_point;
    next_center_point.x = center_traj[3*(nearest_center_idx+1)];
    next_center_point.y = center_traj[3*(nearest_center_idx+1)+1];

    my_point proj_center_point = orhtogonal_proj(nearest_center_point, next_center_point, car_pos);
    
    //use z_wert des kreuzproduktes stattdessen 
    double near_to_next_center_x = next_center_point.x - nearest_center_point.x;
    double near_to_next_center_y = next_center_point.y - nearest_center_point.y;

    double car_to_proj_center_point_x = proj_center_point.x - car_pos.x;
    double car_to_proj_center_point_y = proj_center_point.y - car_pos.y;
    //cross product near_to_next x car_to_proj
    double cross_z = near_to_next_center_x * car_to_proj_center_point_y - near_to_next_center_y *car_to_proj_center_point_x;


    //why smaller then 0? 
    if(cross_z > 0){
        //choose outer border
        border = outer_border;
        border_len = outer_border_len;
        //printf("outer_border as reference\n");
    }
    else{
        //choose inner border
        //even if same, because its gonna be more often the inner border thats the problem, because of time optimization
        border = inner_border;
        border_len = inner_border_len;
        //printf("inner_border as reference\n");
    }

    my_point proj_border_point = find_proj_border_point(border, border_len, car_pos);
   
    //printf("proj_border_point = %f, %f \n", proj_border_point.x, proj_border_point.y);
   
    /* vergleich muss mit proj_center_car erfolgen, 
    um abschätzen zu können, ob wir in der strecke oder außerhalb sind 
    */
    //calculate distance between proj_center_point und proj_border_point  
    typeRNum distance_border_center = euclidian_distance(proj_border_point, proj_center_point);
    typeRNum distance_center_car = euclidian_distance(proj_center_point, car_pos);
    //if((distance_border_center ) <=  distance_center_car){
       //printf("pos contraint not satisfied! distance_border_center = %f, distance_center_car = %f\n", distance_border_center,  distance_center_car);
    //}
    //printf("car_width: %f\n" , car_width);
    
    //das kleinere minus das größere
    out[2] = POW2(distance_center_car) - POW2(distance_border_center - car_width); //abstand auto-centerline < abstand centerline-border - car_width 
    //out[2] = distance_center_car - (distance_border_center - car_width );
    //out[2] = distance_center_car - 0.5; //probe weise Schlauch um die centerline als Constraint

}
/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    //inquality restraints
    UserParam* param = (UserParam*)userparam;

    double* center_traj = param->center_traj;
    int center_traj_len = param->center_traj_len;

    double* outer_border = param->outer_border;
    int outer_border_len = param->outer_border_len;

    double* inner_border = param->inner_border;
    int inner_border_len = param->inner_border_len;

    double* border;
    int border_len;


    my_point car_pos;
    car_pos.x = x[0];
    car_pos.y = x[1]; 

    //project car pos onto centerline
    //my_point proj_center_point = calculate_projected_ref_point(center_traj, center_traj_len, car_pos);

    int nearest_center_idx = getNearestIndex(car_pos.x, car_pos.y,  center_traj, center_traj_len);
    // Coords of nearest point
    my_point nearest_center_point;
    nearest_center_point.x = center_traj[3*nearest_center_idx];
    nearest_center_point.y = center_traj[3*nearest_center_idx + 1];

    my_point next_center_point;
    next_center_point.x = center_traj[3*(nearest_center_idx+1)];
    next_center_point.y = center_traj[3*(nearest_center_idx+1)+1];

    my_point proj_center_point = orhtogonal_proj(nearest_center_point, next_center_point, car_pos);

    //ableitung h nach x mal vector (but why?)
    //output sortiert nach der X[] variable die abgeleitet wird out[0] ^= x[0]'
    //ableitung für quadrierte Contraints
    out[0] = -2* (proj_center_point.x - x[0])* vec[2]; //distanz zur border wird als "pro Aufruf" konstant angenommen und fällt weg
    out[1] = -2* (proj_center_point.y - x[1])* vec[2];

    //ableitung der normalen Constraints
    //out[0] = -(proj_center_point.x - x[0]) /(euclidian_distance(proj_center_point, car_pos))*vec[2];
    //out[1] = -(proj_center_point.y - x[1]) / (euclidian_distance(proj_center_point, car_pos))*vec[2];
   
    out[2] = 0;
    out[3] = vec[0]- vec[1]; //reine optimierung kommt vom gradient based mpc

}
/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Terminal equality constraints gT(T,x(T),p,param,userparam) = 0 
    -------------------------------------------------------- **/
void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Terminal inequality constraints hT(T,x(T),p,param,userparam) <= 0 
    ----------------------------------------------------------- **/
void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dx multiplied by vector vec, i.e. (dhT/dx)^T*vec or vec^T*(dhT/dx) **/
void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dp multiplied by vector vec, i.e. (dhT/dp)^T*vec or vec^T*(dhT/dp) **/
void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dT multiplied by vector vec, i.e. (dhT/dT)^T*vec or vec^T*(dhT/dT) **/
void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Additional functions required for semi-implicit systems 
    M*dx/dt(t) = f(t0+t,x(t),u(t),p) using the solver RODAS,
    see GRAMPC docu for more information
    ------------------------------------------------------- **/
/** Jacobian df/dx in vector form (column-wise) **/
void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dx in vector form (column-wise) **/
void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dt **/
void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian d(dH/dx)/dt  **/
void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *vec, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mfct(typeRNum *out, typeUSERPARAM *userparam)
{
}
/** Transposed mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mtrans(typeRNum *out, typeUSERPARAM *userparam)
{
} 