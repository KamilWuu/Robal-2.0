#ifndef SQUARE_FUNCTION_H
#define SQUARE_FUNCTION_H

typedef struct SquareFunction
{
    double a;
    double b;
    double c;

    double a_d;
}SquareFunction;



void calculateSquareFunctionParams(SquareFunction * square_function, double y1, double d, double h, double z_shift){

    square_function->a = (-4*h)/(d*d);
    square_function->b = ((4*h)/(d*d))*((2*y1) + d);
    square_function->c = (-4*h)/(d*d)*y1*(y1 + d) + z_shift;

    square_function->a_d = square_function->a*2;

}


double getZ_FromY(SquareFunction square_function, double y){
    double z;
    double a, b, c;


    a = square_function.a;
    b = square_function.b;
    c = square_function.c;


    z = (a*(y*y)) + (b*y) + c;

    return z;
}


double getZ_d_FromY(SquareFunction square_function, double y){
    double z;
    double a_d, b;


    a_d = square_function.a_d;
    b = square_function.b;



    z = (a_d * y) + b;

    return z;
}


double getZVelocity_FromYVelocity(SquareFunction square_function, double y, double y_velocity) {
    double z_d = getZ_d_FromY(square_function, y);
    return z_d * y_velocity;
}

#endif //SQUARE_FUNCTION.H