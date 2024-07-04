#ifndef STRUCTS_H
#define STRUCTS_H
#include "header.h"
//***
//***Insertar aqui las definiciones de nuestros contenedores
//***(estructuras)
//***

//La siguiente definicion describe el contenedor que
//usaremos para guardar los parametros de control
//de flujo del programa



struct contenedor_de_parametros_de_control
{
  //Posicion del objeto respecto el sistema
  //de coordenadas de referencia en el instante cero
   double Gr0mX;
   double Gr0mY;
   double Gr0mZ;
   //Orientacion del objeto respecto el sistema
   //de coordenadas de referencia en el instante cero
   double Rr0mX;
   double Rr0mY;
   double Rr0mZ;
   //Traslación del objeto respecto el sistema
  //de coordenadas de referencia en el instante cero al instante uno
   double DTr0a1mX;
   double DTr0a1mY;
   double DTr0a1mZ;
 //Rotación del objeto respecto el sistema
  //de coordenadas de referencia en elinstante cero al instante uno
   double DRr0a1mX;
   double DRr0a1mY;
   double DRr0a1mZ;
   //Traslación del objeto respecto el sistema
   //de coordenadas de referencia en el instante uno al instante dos
   double DTr1a2mX;
   double DTr1a2mY;
   double DTr1a2mZ;

   //Rotación del objeto respecto el sistema
  //de coordenadas de referencia en elinstante uno al instante dos
   double DRr1a2mX;
   double DRr1a2mY;
   double DRr1a2mZ;


   //Posicion del punto H respecto al sistema
   //de coordenadas del objeto
   double Hr0mX;
   double Hr0mY;
   double Hr0mZ;
   // output
   char DirectorioSalida[256];

};

//La siguiente definicion describe el contenedor que
//usaremos para guardar resultados

struct contenedor_de_resultados
{

  //Matriz de rotacion. Dos formas equivalentes de definirla
  double R0[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica para el instante cero
  double R1[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica para el instante uno
  double R2[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica para el instante uno
  double R01[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica para el instante cero a uno
  double R12[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica para el instante uno a dos
  double R02[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica para el instante cero a dos

  //Posicion del punto H respecto al sistema
  //de coordenadas de referencia para mástil del
  //robot en el instante de tiempo k=1
    double HXm1;
    double HYm1;
    double HZm1;
    //Posicion del punto H respecto al sistema
    //de coordenadas de referencia para mástil del
    //robot en el instante de tiempo k=2
    double HXm2;
    double HYm2;
    double HZm2;
    //Posicion del robot respecto al sistema
    //de coordenadas de referencia para el instante 1
    double G1rx;
    double G1ry;
    double G1rz;
    //orientacion del robot respecto al sistema
    //de coordenadas de referencia para el instante 1
    double R1rx;
    double R1ry;
    double R1rz;

    //Posicion del robot respecto al sistema
    //de coordenadas de referencia para el instante 2
    double G2rx;
    double G2ry;
    double G2rz;
    //orientacion del robot respecto al sistema
    //de coordenadas de referencia para el instante 2
    double R2rx;
    double R2ry;
    double R2rz;

    //Traslación del robot respecto al sistema
    //de coordenadas de referencia para el instante cero a dos
    double Tr0a2x;
    double Tr0a2y;
    double Tr0a2z;
    //Rotación del robot respecto al sistema
    //de coordenadas de referencia para el instante cero a dos
    double Rr0a2x;
    double Rr0a2y;
    double Rr0a2z;

};

//***
//***Insertar aqui las definiciones de variables globales,
//***que son aquellas variables que se podran acceder desde
//***cualquier funcion dentro de este archivo
//***

//El siguiente puntero global apuntara al contenedor que
//usaremos para guardar los valores de control de flujo
//del programa que se leeran de un archivo de texto

struct contenedor_de_parametros_de_control *p_parametros;

//El siguiente puntero global apuntara al contenedor
//que usaremos para guardar resultados

struct contenedor_de_resultados *p_resultados;

//La siguiente variable global se usara como contador
//el numero de datos leidos

int numeroDeDatosLeidos=0;

//***
//***Insertar aqui las constantes del programa
//***

#define PI 3.141592652
#define DESPLIEGUE_MATRIZ_DE_ROTACION 1 //1: si, 0: no

#endif
