// ##--------------------------------Main file------------------------------------
// ##
// ## Copyright (C) by Belinda Brown (belindabrownr04@gmail.com)
// POSE estimation, rotation and translation
// ## Computer Vision
// # // Ex#1 
// ##-----------------------------------------------------------------------------

// References
//  First Version created by Dr. Geovanni Martinez on 9/6/17.
//  Copyright © 2017 Dr. Geovanni Martinez. All rights reserved.


//Este programa lee de un archivo de texto llamado current_control_parameters.txt,
//la posicion [GX,GY,GZ]^T y orientacion R=[RX, RY, RX]^T del sistema de coordenadas
//local de un objeto (R,S,T) con respecto al sistema de coordenadas de referencia
//(X,Y,Z), asi como la posicion de un punto [HR,HS,HT]^T del objeto con respecto al
//sistema de coordinadas del objeto (R,S,T). Luego calcula la posicion [HX,HY,HZ]^T de
//ese mismo punto con respecto al sistema de coordenadas de referencia (X,Y,Z). Para
//ello utiliza la transformacion de pose:
//     [HX,HY,HZ]^T=R*[HR,HS,HT]^T+[GX,GY,GZ]^T
//donde R es la matriz de rotacion correspondiente. La matriz de rotacion R y la
//posicion [HX,HY,HZ]^T se visualizan en el terminal y tambien se almacenan en un
//archivo de texto denominado resultados.txt.

//***
//***Incluir aqui los headers (archivos terminados en .h) que
//***contienen los prototipos de las funciones de otras librerias que
//***usaremos en nuestro programa
//***
#include <string.h>
//Se incluyo el siguiente header de la biblioteca
//estandar de C para operaciones de entrada y salida
#include <stdio.h>

//Se incluyo el siguiente header de la biblioteca
//estandar de C para gestion de memoria dinamica,
//control de procesos y otras
#include <stdlib.h>

//Se incluyo el siguiente header debido a que usaremos
//funciones matematicas
#include <math.h>

//***
//***Insertar aqui los prototipos de nuestras funciones
//***

void geoObtenerMatrizDeRotacionR0();
double beconvertirDeRadianesAGrados (double angle);
double geoConvertirDeGradosARadianes (double angle);
void geoLeerParametrosDeControlDeArchivoDeTexto();
void geoSalvarResultadosEnArchivoDeTexto();

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

//Inicio de programa principal
int main()
{
    //definicion de variables locales
    int i; //contador
    double H1[3]; //para guardar temporalmente los resultados de la posicion mastil instante cuno
    double H2[3]; //para guardar temporalmente los resultados de la posicion mastil instante dos
    double P1[6]; //para guardar temporalmente los resultados POSE robor en el instante uno
    double P2[6]; //para guardar temporalmente los resultados POSE robor en el instante dos
    double B02[6]; //para guardar temporalmente los resultados traslacion y rotacion robor en el instante cero a dos
    //Despliegue de autoría en el terminal
    printf("****************************************************************************\n");
    printf("** I EXAMEN DE DISEÑO PROGRAMACIÓN Y PRUEBA, Belinda Brown B61254         **\n");
    printf("** Estimando POSE, rotacion y traslacion                                  **\n");
    printf("** Programa de referencia tomado del Prof. Dr.-Ing. Geovanni Martínez     **\n");
    printf("** IE-0449 Vision por Computador                                          **\n");
    printf("** I-2019                                                                 **\n");
    printf("****************************************************************************\n");
    printf("\n");

    //Reservando e inicializando memoria de contenedor p_parametros
    p_parametros = (struct contenedor_de_parametros_de_control *)malloc(sizeof(struct contenedor_de_parametros_de_control));
    //puntero que direcciona a la posicion y rotacion del objeto respecto el sistema
    //de coordenadas de referencia en el instante cero
    p_parametros->Gr0mX=0.0;
    p_parametros->Gr0mY=0.0;
    p_parametros->Gr0mZ=0.0;
    p_parametros->Rr0mX=0.0;
    p_parametros->Rr0mY=0.0;
    p_parametros->Rr0mZ=0.0;
  //puntero que direcciona a la posicion de memoria para la Traslacion y rotacion del objeto respecto el sistema
  //de coordenadas de referencia en el instante cero al instante uno
    p_parametros->DTr0a1mX=0.0;
    p_parametros->DTr0a1mY=0.0;
    p_parametros->DTr0a1mZ=0.0;
    p_parametros->DRr0a1mX=0.0;
    p_parametros->DRr0a1mY=0.0;
    p_parametros->DRr0a1mZ=0.0;
    //puntero que direcciona a la posicion de memoria para la Traslacion y rotacion del objeto respecto el sistema
    //de coordenadas de referencia en el instante uno al instante dos
    p_parametros->DTr1a2mX=0.0;
    p_parametros->DTr1a2mY=0.0;
    p_parametros->DTr1a2mZ=0.0;
    p_parametros->DRr1a2mX=0.0;
    p_parametros->DRr1a2mY=0.0;
    p_parametros->DRr1a2mZ=0.0;
    //puntero que direcciona a la posicion de memoria para la Posicion del punto H respecto al sistema
    //de coordenadas del objeto
    p_parametros->Hr0mX=0.0;
    p_parametros->Hr0mY=0.0;
    p_parametros->Hr0mZ=0.0;

    //Reservando e inicializando memoria de contenedor p_resultados
    p_resultados = (struct contenedor_de_resultados *)malloc(sizeof(struct contenedor_de_resultados));
    //Inicializando y reservando memoria para posicion de memoria para mástil del
//robot en el instante de tiempo k=1 y k=2
    p_resultados->HXm1=0.0;
    p_resultados->HYm1=0.0;
    p_resultados->HZm1=0.0;
    p_resultados->HXm2=0.0;
    p_resultados->HYm2=0.0;
    p_resultados->HZm2=0.0;

    //Inicializando y reservando memoria para posicion del robot respecto al sistema
    //de coordenadas de referencia para el instante uno y dos
    p_resultados->G1rx=0.0;
    p_resultados->G1ry=0.0;
    p_resultados->G1rz=0.0;
    p_resultados->G2rx=0.0;
    p_resultados->G2ry=0.0;
    p_resultados->G2rz=0.0;
//Inicializando y reservando memoria para posicion de memoria para orientacion del robot respecto al sistema
//de coordenadas de referencia para el instante uno y dos
    p_resultados->R1rx=0.0;
    p_resultados->R1ry=0.0;
    p_resultados->R1rz=0.0;
    p_resultados->R2rx=0.0;
    p_resultados->R2ry=0.0;
    p_resultados->R2rz=0.0;
    //Inicializando y reservando memoria para posicion de memoria para traslacion  del robot respecto al sistema
    //de coordenadas de referencia para el instante uno a dos
    p_resultados->Tr0a2x=0.0;
    p_resultados->Tr0a2y=0.0;
    p_resultados->Tr0a2z=0.0;
    //Inicializando memoria de la matrices de rotacion
    for (i=0;i<9;i++) { //i={0,1,2,3,4,5,6,7,8}
        p_resultados->R0[i]=0.0;//instante cero
        p_resultados->R1[i]=0.0;//instante uno
        p_resultados->R2[i]=0.0;//instante dos
        p_resultados->R01[i]=0.0;//instante cero a uno
        p_resultados->R12[i]=0.0;//instante uni a dos
        p_resultados->R02[i]=0.0;//instante cero a dos
    }

    //Esta funcion lee los parametros de control de flujo del
    //programa desde un archivo de texto y los almacena en el
    //contenedor p_parametros
    geoLeerParametrosDeControlDeArchivoDeTexto();
    //Calculando matriz de rotacion instante base
    geoObtenerMatrizDeRotacionR0();

    //Calculando la posicion del mastil  en instante uno respecto
    //a sistema de coordinadas de referencia
    H1[0]=(p_resultados->R01[0]*(p_parametros->Hr0mX-p_parametros->Gr0mX)+p_resultados->R01[1]*(p_parametros->Hr0mY-p_parametros->Gr0mY)+p_resultados->R01[2]*(p_parametros->Hr0mZ-p_parametros->Gr0mZ))+p_parametros->Gr0mX+p_parametros->DTr0a1mX;
    p_resultados->HXm1=H1[0];
    H1[1]=(p_resultados->R01[3]*(p_parametros->Hr0mX-p_parametros->Gr0mX)+p_resultados->R01[4]*(p_parametros->Hr0mY-p_parametros->Gr0mY)+p_resultados->R01[5]*(p_parametros->Hr0mZ-p_parametros->Gr0mZ))+p_parametros->Gr0mY+p_parametros->DTr0a1mY;
    p_resultados->HYm1=H1[1];
    H1[2]=(p_resultados->R01[6]*(p_parametros->Hr0mX-p_parametros->Gr0mX)+p_resultados->R01[7]*(p_parametros->Hr0mY-p_parametros->Gr0mY)+p_resultados->R01[8]*(p_parametros->Hr0mZ-p_parametros->Gr0mZ))+p_parametros->Gr0mZ+p_parametros->DTr0a1mZ;
    p_resultados->HZm1=H1[2];

    p_resultados->G1rx=p_parametros->Gr0mX+p_parametros->DTr0a1mX;
    p_resultados->G1ry=p_parametros->Gr0mY+p_parametros->DTr0a1mY;
    p_resultados->G1rz=p_parametros->Gr0mZ+p_parametros->DTr0a1mZ;

    //Calculando la posicion del mastil  en instante dos respecto
      //a sistema de coordinadas de referencia
    H2[0]=(p_resultados->R12[0]*(p_resultados->HXm1-p_resultados->G1rx)+p_resultados->R12[1]*(p_resultados->HYm1-p_resultados->G1ry)+p_resultados->R12[2]*(p_resultados->HZm1-p_resultados->G1rz))+p_resultados->G1rx+p_parametros->DTr1a2mX;
    p_resultados->HXm2=H2[0];
    H2[1]=(p_resultados->R12[3]*(p_resultados->HXm1-p_resultados->G1rx)+p_resultados->R12[4]*(p_resultados->HYm1-p_resultados->G1ry)+p_resultados->R12[5]*(p_resultados->HZm1-p_resultados->G1rz))+p_resultados->G1ry+p_parametros->DTr1a2mY;
    p_resultados->HYm2=H2[1];
    H2[2]=(p_resultados->R12[6]*(p_resultados->HXm1-p_resultados->G1rx)+p_resultados->R12[7]*(p_resultados->HYm1-p_resultados->G1ry)+p_resultados->R12[8]*(p_resultados->HZm1-p_resultados->G1rz))+p_resultados->G1rz+p_parametros->DTr1a2mZ;
    p_resultados->HZm2=H2[2];
    //Calculando la POSE robot  en instante de uno  respecto
    //a sistema de coordinadas de referencia
    P1[3]=-asin(p_resultados->R1[6]);
    p_resultados->R1ry=beconvertirDeRadianesAGrados(P1[3]);
    P1[4]=asin(p_resultados->R1[7]/cos(P1[3]));
    p_resultados->R1rx=beconvertirDeRadianesAGrados(P1[4]);
    P1[5]=asin(p_resultados->R1[3]/cos(P1[3]));
    p_resultados->R1rz=beconvertirDeRadianesAGrados(P1[5]);

  //Calculando la POSE robot  en instante de dos respecto
  //a sistema de coordinadas de referencia
    p_resultados->G2rx=p_resultados->G1rx+p_parametros->DTr1a2mX;
    p_resultados->G2ry=p_resultados->G1ry+p_parametros->DTr1a2mY;
    p_resultados->G2rz=p_resultados->G1rz+p_parametros->DTr1a2mZ;

    P2[3]=-asin(p_resultados->R2[6]);
    p_resultados->R2ry=beconvertirDeRadianesAGrados(P2[3]);
    P2[4]=asin(p_resultados->R2[7]/cos(P2[3]));
    p_resultados->R2rx=beconvertirDeRadianesAGrados(P2[4]);
    P2[5]=asin(p_resultados->R2[3]/cos(P2[3]));
    p_resultados->R2rz=beconvertirDeRadianesAGrados(P2[5]);

    //Calculando la traslacion y rotacion robot  en instante de cero a dos respecto
    //a sistema de coordinadas de referencia
    p_resultados->Tr0a2x=p_parametros->DTr0a1mX+p_parametros->DTr1a2mX;
    p_resultados->Tr0a2y=p_parametros->DTr0a1mY+p_parametros->DTr1a2mY;
    p_resultados->Tr0a2z=p_parametros->DTr0a1mZ+p_parametros->DTr1a2mZ;

    B02[3]=-asin(p_resultados->R02[6]);
    p_resultados->Rr0a2y=beconvertirDeRadianesAGrados(B02[3]);
    B02[4]=asin(p_resultados->R02[7]/cos(B02[3]));
    p_resultados->Rr0a2x=beconvertirDeRadianesAGrados(B02[4]);
    B02[5]=asin(p_resultados->R02[3]/cos(B02[3]));
    p_resultados->Rr0a2z=beconvertirDeRadianesAGrados(B02[5]);
    //Salvando los resultados finales en archivo de texto
    geoSalvarResultadosEnArchivoDeTexto();

    printf("\n");
    printf("Resultados:\n");

    printf("La posicion del mástil del robot en el instante de tiempo k=1 con respecto al sistema de coordenadas del mundo \n");
    printf(" HXm1= %.3f\n", p_resultados->HXm1);
    printf(" HYm1= %.3f\n", p_resultados->HYm1);
    printf(" HZm1= %.3f\n", p_resultados->HZm1);

    printf("La posicion del mástil del robot en el instante de tiempo k=2 con respecto al sistema de coordenadas del mundo \n");
    printf(" HXm2= %.3f\n", p_resultados->HXm2);
    printf(" HYm2= %.3f\n", p_resultados->HYm2);
    printf(" HZm2= %.3f\n", p_resultados->HZm2);

    printf("  POSE del robot en k = 1 \n");
    printf("  Posicion del robot con respecto a las coordenadas de referencia del mudno en el instante k=1 \n");
    printf(" G1rx= %.3f\n", p_resultados->G1rx);
    printf(" G1ry= %.3f\n", p_resultados->G1ry);
    printf(" G1rz= %.3f\n", p_resultados->G1rz);
    printf("  Orientacion del robot con respecto a las coordenadas de referencia del mudno en el instante k=1  \n");
    printf(" R1rx= %f\n", p_resultados->R1rx);
    printf(" R1ry= %f\n", p_resultados->R1ry);
    printf(" R1rz= %f\n", p_resultados->R1rz);

    printf("  POSE del robot en k = 2 \n");
    printf(" Posicion del robot con respecto a las coordenadas de referencia del mudno en el instante k=2 \n");
    printf(" G2rx= %.3f\n", p_resultados->G2rx);
    printf(" G2ry= %.3f\n", p_resultados->G2ry);
    printf(" G2rz= %.3f\n", p_resultados->G2rz);
    printf(" Orientacion del robot con respecto a las coordenadas de referencia del mudno en el instante k=2 \n");
    printf(" R2rx= %f\n", p_resultados->R2rx);
    printf(" R2ry= %f\n", p_resultados->R2ry);
    printf(" R2rz= %f\n", p_resultados->R2rz);

    printf("   Traslacion del robor con respecto al sistema de coordenadas del mundo ntre los instantes de tiempo k=0 y k=2 \n");
    printf(" Tr0a2x= %f\n", p_resultados->Tr0a2x);
    printf(" Tr0a2y= %f\n", p_resultados->Tr0a2y);
    printf(" Tr0a2z= %f\n", p_resultados->Tr0a2z);
    printf("   Rotacion del robot con respecto al sistema de coordenadas del mundo ntre los instantes de tiempo k=0 y k=2  \n");
    printf(" Rr0a2x= %f\n", p_resultados->Rr0a2x);
    printf(" Rr0a2y= %f\n", p_resultados->Rr0a2y);
    printf(" Rr0a2z= %f\n", p_resultados->Rr0a2z);

    //Liberando memoria reservada manualmente
    free(p_parametros);
    free(p_resultados);

    return 0;
}
//Fin de programa principal

//*******************************************************
//*******************************************************
//***** Introduzca aqui sus funciones               *****
//*******************************************************
//*******************************************************

void geoObtenerMatrizDeRotacionR0()
{
    double RX, RY, RZ;

    //Conviertiendo angulos a radianes
    RX=geoConvertirDeGradosARadianes(p_parametros->Rr0mX);
    RY=geoConvertirDeGradosARadianes(p_parametros->Rr0mY);
    RZ=geoConvertirDeGradosARadianes(p_parametros->Rr0mZ);

    //Calculando la matriz de rotacion de k=0
    p_resultados->R0[0]=cos(RY)*cos(RZ);
    p_resultados->R0[1]=sin(RX)*sin(RY)*cos(RZ)-cos(RX)*sin(RZ);
    p_resultados->R0[2]=cos(RX)*sin(RY)*cos(RZ)+sin(RX)*sin(RZ);
    p_resultados->R0[3]=cos(RY)*sin(RZ);
    p_resultados->R0[4]=sin(RX)*sin(RY)*sin(RZ)+cos(RX)*cos(RZ);
    p_resultados->R0[5]=cos(RX)*sin(RY)*sin(RZ)-sin(RX)*cos(RZ);
    p_resultados->R0[6]=-sin(RY);
    p_resultados->R0[7]=sin(RX)*cos(RY);
    p_resultados->R0[8]=cos(RX)*cos(RY);



    //Conviertiendo angulos a radianes
    RX=geoConvertirDeGradosARadianes(p_parametros->DRr0a1mX);
    RY=geoConvertirDeGradosARadianes(p_parametros->DRr0a1mY);
    RZ=geoConvertirDeGradosARadianes(p_parametros->DRr0a1mZ);

    //Calculando la matriz de rotacion
    p_resultados->R01[0]=cos(RY)*cos(RZ);
    p_resultados->R01[1]=sin(RX)*sin(RY)*cos(RZ)-cos(RX)*sin(RZ);
    p_resultados->R01[2]=cos(RX)*sin(RY)*cos(RZ)+sin(RX)*sin(RZ);
    p_resultados->R01[3]=cos(RY)*sin(RZ);
    p_resultados->R01[4]=sin(RX)*sin(RY)*sin(RZ)+cos(RX)*cos(RZ);
    p_resultados->R01[5]=cos(RX)*sin(RY)*sin(RZ)-sin(RX)*cos(RZ);
    p_resultados->R01[6]=-sin(RY);
    p_resultados->R01[7]=sin(RX)*cos(RY);
    p_resultados->R01[8]=cos(RX)*cos(RY);

      //Calculando la matriz de rotacion de cero a uno

    p_resultados->R1[0]=p_resultados->R01[0]*p_resultados->R0[0]+p_resultados->R01[1]*p_resultados->R0[3]+ p_resultados->R01[2]*p_resultados->R0[6];
    p_resultados->R1[1]=p_resultados->R01[0]*p_resultados->R0[1]+p_resultados->R01[1]*p_resultados->R0[4]+ p_resultados->R01[2]*p_resultados->R0[7];
    p_resultados->R1[2]=p_resultados->R01[0]*p_resultados->R0[2]+p_resultados->R01[1]*p_resultados->R0[5]+ p_resultados->R01[2]*p_resultados->R0[8];
    p_resultados->R1[3]=p_resultados->R01[3]*p_resultados->R0[0]+p_resultados->R01[4]*p_resultados->R0[3]+ p_resultados->R01[5]*p_resultados->R0[6];
    p_resultados->R1[4]=p_resultados->R01[3]*p_resultados->R0[1]+p_resultados->R01[4]*p_resultados->R0[4]+ p_resultados->R01[5]*p_resultados->R0[7];
    p_resultados->R1[5]=p_resultados->R01[3]*p_resultados->R0[2]+p_resultados->R01[4]*p_resultados->R0[5]+ p_resultados->R01[5]*p_resultados->R0[8];
    p_resultados->R1[6]=p_resultados->R01[6]*p_resultados->R0[0]+p_resultados->R01[7]*p_resultados->R0[3]+ p_resultados->R01[8]*p_resultados->R0[6];
    p_resultados->R1[7]=p_resultados->R01[6]*p_resultados->R0[1]+p_resultados->R01[7]*p_resultados->R0[4]+ p_resultados->R01[8]*p_resultados->R0[7];
    p_resultados->R1[8]=p_resultados->R01[6]*p_resultados->R0[2]+p_resultados->R01[7]*p_resultados->R0[5]+ p_resultados->R01[8]*p_resultados->R0[8];


    RX=geoConvertirDeGradosARadianes(p_parametros->DRr1a2mX);
    RY=geoConvertirDeGradosARadianes(p_parametros->DRr1a2mY);
    RZ=geoConvertirDeGradosARadianes(p_parametros->DRr1a2mZ);

      //Calculando la matriz de rotacion de uno a dos
    p_resultados->R12[0]=cos(RY)*cos(RZ);
    p_resultados->R12[1]=sin(RX)*sin(RY)*cos(RZ)-cos(RX)*sin(RZ);
    p_resultados->R12[2]=cos(RX)*sin(RY)*cos(RZ)+sin(RX)*sin(RZ);
    p_resultados->R12[3]=cos(RY)*sin(RZ);
    p_resultados->R12[4]=sin(RX)*sin(RY)*sin(RZ)+cos(RX)*cos(RZ);
    p_resultados->R12[5]=cos(RX)*sin(RY)*sin(RZ)-sin(RX)*cos(RZ);
    p_resultados->R12[6]=-sin(RY);
    p_resultados->R12[7]=sin(RX)*cos(RY);
    p_resultados->R12[8]=cos(RX)*cos(RY);

  //Calculando la matriz de rotacion de dos

    p_resultados->R2[0]=p_resultados->R12[0]*p_resultados->R1[0]+p_resultados->R12[1]*p_resultados->R1[3]+ p_resultados->R12[2]*p_resultados->R1[6];
    p_resultados->R2[1]=p_resultados->R12[0]*p_resultados->R1[1]+p_resultados->R12[1]*p_resultados->R1[4]+ p_resultados->R12[2]*p_resultados->R1[7];
    p_resultados->R2[2]=p_resultados->R12[0]*p_resultados->R1[2]+p_resultados->R12[1]*p_resultados->R1[5]+ p_resultados->R12[2]*p_resultados->R1[8];
    p_resultados->R2[3]=p_resultados->R12[3]*p_resultados->R1[0]+p_resultados->R12[4]*p_resultados->R1[3]+ p_resultados->R12[5]*p_resultados->R1[6];
    p_resultados->R2[4]=p_resultados->R12[3]*p_resultados->R1[1]+p_resultados->R12[4]*p_resultados->R1[4]+ p_resultados->R12[5]*p_resultados->R1[7];
    p_resultados->R2[5]=p_resultados->R12[3]*p_resultados->R1[2]+p_resultados->R12[4]*p_resultados->R1[5]+ p_resultados->R12[5]*p_resultados->R1[8];
    p_resultados->R2[6]=p_resultados->R12[6]*p_resultados->R1[0]+p_resultados->R12[7]*p_resultados->R1[3]+ p_resultados->R12[8]*p_resultados->R1[6];
    p_resultados->R2[7]=p_resultados->R12[6]*p_resultados->R1[1]+p_resultados->R12[7]*p_resultados->R1[4]+ p_resultados->R12[8]*p_resultados->R1[7];
    p_resultados->R2[8]=p_resultados->R12[6]*p_resultados->R1[2]+p_resultados->R12[7]*p_resultados->R1[5]+ p_resultados->R12[8]*p_resultados->R1[8];

//Calculando la matriz de rotacion de cero a dos
    p_resultados->R02[0]=p_resultados->R2[0]*p_resultados->R0[0]+p_resultados->R2[1]*p_resultados->R0[1]+ p_resultados->R2[2]*p_resultados->R0[2];
    p_resultados->R02[1]=p_resultados->R2[0]*p_resultados->R0[3]+p_resultados->R2[1]*p_resultados->R0[4]+ p_resultados->R2[2]*p_resultados->R0[5];
    p_resultados->R02[2]=p_resultados->R2[0]*p_resultados->R0[6]+p_resultados->R2[1]*p_resultados->R0[7]+ p_resultados->R2[2]*p_resultados->R0[8];
    p_resultados->R02[3]=p_resultados->R2[3]*p_resultados->R0[0]+p_resultados->R2[4]*p_resultados->R0[1]+ p_resultados->R2[5]*p_resultados->R0[2];
    p_resultados->R02[4]=p_resultados->R2[3]*p_resultados->R0[3]+p_resultados->R2[4]*p_resultados->R0[4]+ p_resultados->R2[5]*p_resultados->R0[5];
    p_resultados->R02[5]=p_resultados->R2[3]*p_resultados->R0[6]+p_resultados->R2[4]*p_resultados->R0[7]+ p_resultados->R2[5]*p_resultados->R0[8];
    p_resultados->R02[6]=p_resultados->R2[6]*p_resultados->R0[0]+p_resultados->R2[7]*p_resultados->R0[1]+ p_resultados->R2[8]*p_resultados->R0[2];
    p_resultados->R02[7]=p_resultados->R2[6]*p_resultados->R0[3]+p_resultados->R2[7]*p_resultados->R0[4]+ p_resultados->R2[8]*p_resultados->R0[5];
    p_resultados->R02[8]=p_resultados->R2[6]*p_resultados->R0[6]+p_resultados->R2[7]*p_resultados->R0[7]+ p_resultados->R2[8]*p_resultados->R0[8];

}

double geoConvertirDeGradosARadianes(double angle)
{
    double res;

    res=angle*PI/180.0;

    return(res);
}

double beconvertirDeRadianesAGrados(double angle)
{
    double res;

    res=angle*180.0/PI;

    return(res);
}

void geoLeerParametrosDeControlDeArchivoDeTexto()
{
    FILE *archivo;
    char d1[256], d2[256], d3[256];

    printf("Leyendo los datos de entrada:\n");

    //Abriendo archivo en mode de lectura
    char nombreDeArchivo[256]="current_control_parameters.txt";
    archivo = fopen(nombreDeArchivo, "r");
    if (!archivo) {
        printf("No se pudo abrir el archivo: current_control_parameters.txt\n");
        exit(1);
    }

    //Leyendo datos linea por linea

    //Brincando la primera y segunda lineas
    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    fscanf(archivo, "\n");

    printf("  Posicion del robot con respecto al sistema de coordenadas en el instante k = 0\n");
    //Leyendo Gr0mX
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Gr0mX=(double)atof(d3);
    printf("   Gr0mX: %.2f\n", p_parametros->Gr0mX);
    numeroDeDatosLeidos++;
    //Leyendo Gr0mY
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Gr0mY=(double)atof(d3);
    printf("   Gr0mY: %.2f\n", p_parametros->Gr0mY);
    numeroDeDatosLeidos++;
    //Leyendo Gr0mZ
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Gr0mZ=(double)atof(d3);
    printf("   Gr0mZ: %.2f\n", p_parametros->Gr0mZ);
    numeroDeDatosLeidos++;


    //Brincando linea de texto
    fscanf(archivo, "\n");

    printf("  Orientacion del robot con respecto al sistema de coordenadas en el instante k = 0\n");
    //Leyendo Rr0mX
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Rr0mX=(double)atof(d3);
    printf("   Rr0mX: %.2f\n", p_parametros->Rr0mX);
    numeroDeDatosLeidos++;
    //Leyendo Rr0mY
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Rr0mY=(double)atof(d3);
    printf("   Rr0mY: %.2f\n", p_parametros->Rr0mY);
    numeroDeDatosLeidos++;
    //Leyendo Rr0mZ
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Rr0mZ=(double)atof(d3);
    printf("   Rr0mZ: %.2f\n", p_parametros->Rr0mZ);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    fscanf(archivo, "\n");

    printf("   Traslacion del robot con respecto al sistema de coordenadas del instante k=0 al k=1\n");
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DTr0a1mX=(double)atof(d3);
    printf("   DTr0a1mX: %.2f\n", p_parametros->DTr0a1mX);
    numeroDeDatosLeidos++;
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DTr0a1mY=(double)atof(d3);
    printf("   DTr0a1mY: %.2f\n", p_parametros->DTr0a1mY);
    numeroDeDatosLeidos++;
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DTr0a1mZ=(double)atof(d3);
    printf("   DTr0a1mZ: %.2f\n", p_parametros->DTr0a1mZ);
    numeroDeDatosLeidos++;

    fscanf(archivo, "\n");

    printf("   Orientacion del robot con respecto al sistema de coordenadas del instante k=0 al k=1\n");
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DRr0a1mX=(double)atof(d3);
    printf("   DRr0a1mX: %.2f\n", p_parametros->DRr0a1mX);
    numeroDeDatosLeidos++;
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DRr0a1mY=(double)atof(d3);
    printf("   DRr0a1mY: %.2f\n", p_parametros->DRr0a1mY);
    numeroDeDatosLeidos++;
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DRr0a1mZ=(double)atof(d3);
    printf("   DRr0a1mZ: %.2f\n", p_parametros->DRr0a1mZ);
    numeroDeDatosLeidos++;

    fscanf(archivo, "\n");

    printf("  Traslacion del robot con respecto al sistema de coordenadas del instante k=1 al k=2\n");
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DTr1a2mX=(double)atof(d3);
    printf("   DTr1a2mX: %.2f\n", p_parametros->DTr1a2mX);
    numeroDeDatosLeidos++;
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DTr1a2mY=(double)atof(d3);
    printf("   DTr1a2mY: %.2f\n", p_parametros->DTr1a2mY);
    numeroDeDatosLeidos++;
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DTr1a2mZ=(double)atof(d3);
    printf("   DTr1a2mZ: %.2f\n", p_parametros->DTr1a2mZ);
    numeroDeDatosLeidos++;

    fscanf(archivo, "\n");

    printf("  Orientacion del robot con respecto al sistema de coordenadas del instante k=1 al k=2\n");
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DRr1a2mX=(double)atof(d3);
    printf("   DRr1a2mX: %.2f\n", p_parametros->DRr1a2mX);
    numeroDeDatosLeidos++;
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DRr1a2mY=(double)atof(d3);
    printf("   DRr1a2mY: %.2f\n", p_parametros->DRr1a2mY);
    numeroDeDatosLeidos++;
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->DRr1a2mZ=(double)atof(d3);
    printf("   DRr1a2mZ: %.2f\n", p_parametros->DRr1a2mZ);
    numeroDeDatosLeidos++;

    fscanf(archivo, "\n");

    printf("La posicion del mástil del robot en el instante de tiempo k=0 con respecto al sistema de coordenadas del mundo\n");
    //Leyendo Hr0mX
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Hr0mX=(double)atof(d3);
    printf("   Hr0mX: %.2f\n", p_parametros->Hr0mX);
    numeroDeDatosLeidos++;
    //Leyendo Hr0mY
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Hr0mY=(double)atof(d3);
    printf("   Hr0mY: %.2f\n", p_parametros->Hr0mY);
    numeroDeDatosLeidos++;
    //Leyendo Hr0mZ
    fscanf(archivo, "%s %s\n", d1, d3);
    p_parametros->Hr0mZ=(double)atof(d3);
    printf("   Hr0mZ: %.2f\n", p_parametros->Hr0mZ);
    numeroDeDatosLeidos++;

    fscanf(archivo, "\n");

    fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    strcpy(p_parametros->DirectorioSalida,d3);
    printf("   Directorio de salida: %s\n", p_parametros->DirectorioSalida);
    numeroDeDatosLeidos++;
    printf("  Numero de datos leidos: %d\n", numeroDeDatosLeidos);


    //Cerrando archivo
    fclose(archivo);
}

void geoSalvarResultadosEnArchivoDeTexto()
{
    FILE *archivo;

    char nombreArchivo[256]="resultados.txt";
    char rutaSalida[256];
    strcpy(rutaSalida,p_parametros->DirectorioSalida);

    archivo = fopen(strcat(rutaSalida, nombreArchivo), "w");
    if (!archivo) {
        printf("No se pudo abrir el archivo: resultados.txt\n");
        exit(1);
    }

    //Guardando datos
    fprintf(archivo, "Posicion del mastil con respecto al sistema de coordenadas del mundo en el instante 1 \n");
    fprintf(archivo, "HXm1= %.2f\n", p_resultados->HXm1);
    fprintf(archivo, "HYm1= %.2f\n", p_resultados->HYm1);
    fprintf(archivo, "HZm1= %.2f\n", p_resultados->HZm1);

    fprintf(archivo, "Posicion del mastil con respecto al sistema de coordenadas del mundo en el instante 2 \n");
    fprintf(archivo, "HXm2= %.2f\n", p_resultados->HXm2);
    fprintf(archivo, "HYm2= %.2f\n", p_resultados->HYm2);
    fprintf(archivo, "HZm2= %.2f\n", p_resultados->HZm2);

    fprintf(archivo, "Posicion del robot con respecto al sistema de coordenadas del mundo  en el instante 1 \n");
    fprintf(archivo, "G1rx= %.2f\n", p_resultados->G1rx);
    fprintf(archivo, "G1ry= %.2f\n", p_resultados->G1ry);
    fprintf(archivo, "G1rz= %.2f\n", p_resultados->G1rz);

    fprintf(archivo, "Orientacion del robot con respecto al sistema de coordenadas del mundo en el instante 1 \n");
    fprintf(archivo, "R1rx= %f\n", p_resultados->R1rx);
    fprintf(archivo, "R1ry= %f\n", p_resultados->R1ry);
    fprintf(archivo, "R1rz= %f\n", p_resultados->R1rz);


    fprintf(archivo, "Posicion del robot con respecto al sistema de coordenadas del mundo en el instante 2 \n");
    fprintf(archivo, "G2rx= %.2f\n", p_resultados->G2rx);
    fprintf(archivo, "G2ry= %.2f\n", p_resultados->G2ry);
    fprintf(archivo, "G2rz= %.2f\n", p_resultados->G2rz);

    fprintf(archivo, "Orientacion del robot con respecto al sistema de coordenadas del mundo en el instante 2 \n");
    fprintf(archivo, "R2rx= %f\n", p_resultados->R2rx);
    fprintf(archivo, "R2ry= %f\n", p_resultados->R2ry);
    fprintf(archivo, "R2rz= %f\n", p_resultados->R2rz);

    fprintf(archivo, "Traslacion del robot con respecto al sistema de coordenadas del mundo en el instante 0 a 2 \n");
    fprintf(archivo, "Tr0a2x= %.2f\n", p_resultados->Tr0a2x);
    fprintf(archivo, "Tr0a2y= %.2f\n", p_resultados->Tr0a2y);
    fprintf(archivo, "Tr0a2z= %.2f\n", p_resultados->Tr0a2z);

    fprintf(archivo, "Rotacion del robot con respecto al sistema de coordenadas del mundo en el instante 0 a 2 \n");
    fprintf(archivo, "Rr0a2x= %f\n", p_resultados->Rr0a2x);
    fprintf(archivo, "Rr0a2y= %f\n", p_resultados->Rr0a2y);
    fprintf(archivo, "Rr0a2z= %f\n", p_resultados->Rr0a2z);

    //Cerrando archivo
    fclose(archivo);
}
