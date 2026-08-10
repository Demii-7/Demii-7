//
//  thermal_dist_simulator.cpp
//
//  Thermal Distribution Simulation
//
//  Created by Demarce Williams on 08/11/2023.
//
//  Description: The program simulates thermal dissipation on a metal plate of isothermal boundary temperatures given a stimulation temperature and a point on the plate and produce a thermal image of the plate after thermal distribution./*___________________________________________________________________________________________________________________________*/

//LIbraries
#include <iostream>
#include <iomanip>
#include "Bitmap_Helper.h"
#include <cstdint>
#include <string>
#include <fstream>
#include <cmath>
#include <limits>

using namespace std;

//Fuunction Prototypes
double** create2DGrid(int& ROWS, int& COLS);    //Creates 2D array to store temperature values
void deallocateDynamicArray(double** arr, int ROWS);// Dynamically deallocates memory
void displayGrid(double** arr, int ROWS, int COLS); //Display 2D Array Elements in a Grid format
void setBoundaryConditions(double** arr, int ROWS, int COLS); //Set Isothermal Boundary temperatures
void stimulateGrid(double** arr, int ROWS, int COLS, int& x, int& y, int& constant); //Stimulate a point on the grid witha  specific temperature
void simulateThermalDissipation(double** arr, int ROWS, int COLS, int x, int y, int constant); //Simulate thermal Distribution
void thermalImage(double** arr, int ROWS, int COLS); //Write thermal image in file
bool isValidInput(); //Checks validity of integer input

int main() {
    
    //Declare Objects
    double** arr = NULL;        //Double pointer to two dimensional Array
    int COLS(0), ROWS(0);       //Stores column and row size of grid
    int choice(0);              // Stores user selection from menu
    int constant(0); //Stores user choice: 1 for keeping constant stimulation temperature and 0 for changing it
    int x(0), y(0); //Stores stimulation coordinates, x- columns, y-rows
    
    //Set precision of console output
    cout << fixed << setprecision(2);

    
    //Main menu loop
    do{
        // Program title and Welcome message
        cout << "\n                     THERMAL DISTRIBUTION SIMULATION\n";
        cout << "\n              Welcome to the Thermal Distribution Simulator\n";
        
        // Prints menu to user
        cout << "\n1  : Create 2D Grid.\n";
        cout << "2  : Set Boundary Conditions.\n";
        cout << "3  : Stimulate the Grid.\n";
        cout << "4  : Simulate the Thermal Dissipation.\n";
        cout << "5  : Display Grid\n";
        cout << "6  : Exit Program.\n";

        
        //Ensures user input is of integer type
        do{
            // Prompts the user to make a selection from menu
            cout << "\nPlease make a selection from the menu above: ";
            cin >> choice;
                        
        }while(!isValidInput());
        
        //Switch structure to process user selection
        switch(choice){
            case 1:
                //Deallocate memory if user chooses to create new array
                deallocateDynamicArray(arr, ROWS);

                //Creates 2D Array and stores it's address in arr pointer
                arr = create2DGrid(ROWS, COLS);
                break;
            case 2:{
                //Ensures array is created before proceeding to this option
                if(arr == NULL){
                    cout << "\nERROR! Please Create 2D Grid first!\n";
                    continue;
                }//End if
                
                //Set boundary conditions for grid boundaries
                setBoundaryConditions(arr, ROWS, COLS);
                break;
            }case 3:{
                //Ensures array is created before proceeding to this option
                if(arr == NULL){
                    cout << "\nERROR! Please Create 2D Grid first!\n";
                    continue;
                }//End if
                
                //Set stimulation temperature
                stimulateGrid(arr, ROWS, COLS, x, y, constant);
                break;
            }case 4:{
                //Ensures array is created before proceeding to this option
                if(arr == NULL){
                    cout << "\nERROR! Please Create 2D Grid first!\n";
                    continue;
                }//End if
                
                //Thermal distribution Simulation
                simulateThermalDissipation(arr, ROWS, COLS, x, y, constant);
                break;
            }case 5:{
                //Ensures array is created before proceeding to this option
                if(arr == NULL){
                    cout << "\nERROR! Please Create 2D Grid first!\n";
                    continue;
                }//End if
                
                //Dispalys the 2d Grid of temperature values
                displayGrid(arr, ROWS, COLS);
                break;
            }case 6:{
                //Exit messages
                cout << "\nThank you for using our system...\n";
                cout << "Goodbye!\n" << endl;
                break;
            }default:{
                cout << "\n\nError! Please choose an option in the menu!!\n\n";
                continue;
            }//End of case 2 block
        }//End of switch statement
        
    }while(choice != 6); // End of main menu loop
    //Exit Program
    
    //Dynamically deallocate memory
    deallocateDynamicArray(arr, ROWS);
    
    return 0;
    
}//End Main

//FUNCTION DEFINITIONS

double** create2DGrid(int& ROWS, int& COLS){
    
    //Prompt user to enter row and column size
    //The dimensions are limited the dimensions to 4 minimum as to ensure there are at least 4 internal points.
    do{
        cout << "\nEnter the row size of the grid: ";
        cin >> ROWS;
        //Error message for negative input
        if( ROWS < 3){
            cout << "\nError! Please enter a positive number greater than 3!"<< endl;
        }//End if
        
    }while(!isValidInput() || ROWS < 3);
    
    do{
        cout << "\nEnter the column size of the grid: ";
        cin >> COLS;
        //Error message for negative input
        if( COLS < 3){
            cout << "\nError! Please enter a positive number greater than 3!"<< endl;
        }//End if

    }while(!isValidInput() || COLS < 3);
    
    //Create 2D Grid
    //Declare array of pointers
    double** arr = new double*[ROWS];
    
    //Declare 2D array
    for(int i = 0; i < ROWS; i++){
        //Assign 1D array to each pointer element
        arr[i] = new double[COLS];
    }//End for loop
    
    //Initial 2D Grid values to 0
    for(int i = 0; i < ROWS; i++){
        for(int j = 0; j < COLS; j++){
            arr[i][j] = 0;
        }// end inner for loop
    }//End outer for loop
    
    cout << "\nThe 2D Grid was created successfully!" << endl;
    //Return address of two dimensional array
    return arr;
}//end of function

void deallocateDynamicArray(double** arr, int ROWS){
    //Dynamic Memory Deallocation
    //Deallocate inner array
    for(int i = 0; i < ROWS; i++){
        delete[] arr[i];
    }//End for loop
    //Deallocate array of pointers
    delete[] arr;
}//End of function


void displayGrid (double** arr, int ROWS, int COLS){
    for(int i = 0; i < ROWS; i++){
        for(int j = 0; j < COLS; j++){
            //Outputr temperature values to console
            cout << setw(7) << arr[i][j];
        }//End inner for loop
        cout << endl;
    }//End outer for loop
}//End of function

void setBoundaryConditions(double** arr, int ROWS, int COLS){
    //Declare Array to store boundary values
    double boundary[4] = {0};
    
    //Prompt user to enter boundary values
    do{
        cout << "\nEnter top boundary temperature between 0 and 255: ";
        cin >> boundary[0];
        //Error message for out of range input
        if(boundary[0] < 0 || boundary[0] > 255){
            cout << "Error! Please enter a number between 0 and 255, inclusive!"<< endl;
        }//End if
    }while(!isValidInput() || boundary[0] < 0 || boundary[0] > 255);
    
    do{
        cout << "\nEnter bottom boundary temperature between  0 and 255: ";
        cin >> boundary[1];
        //Error message for out of range input
        if(boundary[1] < 0 || boundary[1] > 255){
            cout << "Error! Please enter a number between 0 and 255, inclusive!"<< endl;
        }//End if

    }while(!isValidInput() || boundary[1] < 0 || boundary[1] > 255);
    
    do{
        cout << "\nEnter left boundary temperature between 0 and 255: ";
        cin >> boundary[2];
        //Error message for out of range input
        if(boundary[2] < 0 || boundary[2] > 255){
            cout << "Error! Please enter a number between 0 and 255, inclusive!"<< endl;
        }//End if
        
    }while(!isValidInput() || boundary[2] < 0 || boundary[2] > 255);
    
    do{
        cout << "\nEnter right boundary temperature between 0 and 255: ";
        cin >> boundary[3];
        //Error message for out of range input
        if(boundary[3] < 0 || boundary[3] > 255){
            cout << "Error! Please enter a number between 0 and 255, inclusive!"<< endl;
        }//End if

    }while(!isValidInput() || boundary[3] < 0 || boundary[3] > 255);
    
    //Initialise Top and Bottom Boundary Temperatures
    for(int j = 0; j < COLS; j++){
        arr[0][j] = boundary[0];
        arr[ROWS-1][j] = boundary[1];
    }
    //Initialise Left and Right Boundary Temperatures
    for(int i = 1; i < ROWS-1; i++){
        arr[i][0] = boundary[2];
        arr[i][COLS-1] = boundary[3];
    }//End of for loop
    
    cout << "\nThe boundary conditions were set successfully!" << endl;
    
}//End of function

void stimulateGrid(double** arr, int ROWS, int COLS, int& x, int& y, int& constant){
    
    //Declare stimulation temperature object
    double stimulate(0);
    
    //Prompt user to enter coordinates of stimulation point
    do{
        cout << "\nEnter the row coordinate of the stimulation point: ";
        cin >> y;
        //Error message for out of range input
        if( y <= 0 || y >= ROWS-1){
            cout << "Error! Coordinate out of range!"<< endl;
        }//End if

    }while(!isValidInput() || y <= 0 || y >= ROWS-1);
    
    do{
        cout << "\nEnter the column coordinate of the stimulation point: ";
        cin >> x;
        //Error message for out of range input
        if( x <= 0 || x >= COLS-1){
            cout << "Error! Coordinate out of range!"<< endl;
        }//End if

    }while(!isValidInput() || x <= 0 || x >= COLS-1);
    
    //Prompt user to enter stimulation value
    do{
        cout << "\nEnter stimulation value: ";
        cin >> stimulate;
        //Error message for out of range input
        if( stimulate < 0 || stimulate > 255){
            cout << "Error! Please enter a number between 0 and 255, inclusive!"<< endl;
        }//End if
    }while(!isValidInput() || stimulate < 0 || stimulate > 255);
    
    //Prompt the user to choose whether to keep stimulation value constant
    do{
        cout << "\n Do you wish to keep the stimulation temperature constant?\n  1: Yes\n  0: No\n Select: ";
        cin >> constant;
        //Error message for out of range input
        if(constant < 0 || constant > 1){
            cout << "Error! Please enter 0 or 1 ONLY!"<< endl;
        }//End if
    }while(!isValidInput() || constant < 0 || constant > 1);

    
    //Assign stimulation value to coordinate
    arr[y][x] = stimulate;
    
    cout << "\nThe stimulation temperature was set successfully!" << endl;
    
}//End of function

void simulateThermalDissipation(double** arr, int ROWS, int COLS, int x, int y, int constant){
    
    //Declare Objects
    double tolerance(0); // stores tolerance value
    bool indicator = true; //indicates whether the grid values are below the tolerance value
    
    //Prompt user to enter tolerance value
    do{
        cout << "\nEnter the tolerance value: ";
        cin >> tolerance;
        //Error message for out of range input
        //Limit user input to tolerance values less than 10 but greater than 1
        if( tolerance < 0 || tolerance > 10){
            cout << "Error! Please enter a number between 0 and 10, inclusive!"<< endl;
        }//End if

    }while(!isValidInput() || tolerance < 0 || tolerance > 10);

    //Theraml dissipation simulation until all temperature changes are less than tolerance
    do{
        //Set indicator to true
        indicator = true;
        
        //Update Grid value until all are below tolerance value
        for(int i = 1; i < ROWS-1; i++){
            for(int j = 1; j < COLS-1; j++){
                
                //Skips Stimulation point if user desides to keep it constant
                if(constant == 1 && i == y && j == x){
                    continue;
                }
                    
                //Store current temperature value
                double old_temp = arr[i][j];
                //Claculate and store new temperature value based on
                double new_temp = (arr[i][j-1] + arr[i][j+1] + arr[i-1][j] + arr[i+1][j])/4.0;
                    
                //Check if temperature change falls below tolerance value
                if(fabs(old_temp - new_temp) >= tolerance){
                    //Set indicator to false
                    indicator = false;
                    //Update the value at position [i][j] with average of surrounding values
                    arr[i][j] = new_temp;
                }// End if
            }//End inner for loop
        }//End outer for loop
    }while(!indicator);//End while loop
    
    thermalImage(arr, ROWS, COLS);
        
    cout << "Thermal distribition  simulation was successfully completed!" << endl;
}//End of function


void thermalImage(double** arr, int ROWS, int COLS){
    
    double min(arr[0][0]), max(0);
    
    //Check for minimunm and maaximun temperatures
    for(int i = 0; i < ROWS; i++){
        for(int j = 0; j < COLS; j++){
            //Check for maaximun temperatures
            if(arr[i][j] > max){
                max = arr[i][j];
            }
            //Check for minimunm temperatures
            if(arr[i][j] < min ){
                min = arr[i][j];
            }

        }//End inner for loop
    }//End outer for loop
    
    //Create unsigned inetger array
    uint8_t** array = new uint8_t *[ROWS];
    
    //Deallocate inner array
    for(int i = 0; i < ROWS; i++){
        array[i] = new uint8_t [COLS];
    }//End for loop
    
    //Calculate current temperature range
    double range = max - min;
    //Variables to map current temperature values to new range (0-255);
    double old_val, new_val, new_range(255);
    
    //Map temperature values to range (0-255)
    for(int i = 0; i < ROWS; i++){
        for(int j = 0; j < COLS; j++){
            //Store currentr temperature value
            old_val = arr[i][j];
            
            //Map current temperature value to new range (0-255)
            new_val = (old_val - min) * new_range/ range + 0.5;
            
            //Cast double values to unsigned integer and store them into the new array
            //Populate Unisgned integer array
            array[i][j] = static_cast<uint8_t>(new_val);
            
        }//End inner for loop
    }//End outer for loop

    //Create Thermal Image using Thermal Temperature data
    writeBitmap("thermal_image.bmp",array, COLS, ROWS);
    
    //Dynamic Memory Deallocation
    //Deallocate inner array
    for(int i = 0; i < ROWS; i++){
        delete[] array[i];
    }//End for
    //Deallocate array of pointers
    delete[] array;
    
}//End of function


bool isValidInput(){
    bool validInput = true;
    
    //Checks Validity of input
    if(cin.fail()){
        
        validInput = false;
        cout << "\nError! Invalid input! Please enetr a valid input!\n";
        
        //clears characters in i
        //Clear input buffer up to the limit or upon encountering a newline character.
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(),'\n');
    }//End if
    
    //Return outcome of input validation true- valid false- invalid
    return validInput;
}//End of function
