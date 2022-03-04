#include "GalilController.h"

#include <algorithm>



GalilController::GalilController(GCStringIn ipAddress)
{
    GOpen(ipAddress, &m_gc); // open the connection
    
    
} // GalilController Constructor

GalilController::~GalilController()
{
    GClose(m_gc);
    
} // GalilController destructor

GCStringOut GalilController::bufferToGCStringOut(char* buffer, unsigned int buffer_size)
{
    GCStringOut stringout = new char[GALIL_BUFFER_SIZE];
    
    memcpy(stringout, buffer, buffer_size);
    
    return stringout;
    
} // GalilController::bufferToGCStringOut

GCStringOut GalilController::command(GCStringIn command)
{
    e(GCmdT(m_gc, command, m_buffer, GALIL_BUFFER_SIZE, NULL)); // trimmed version
    // this->e(GCommand(m_gc, command, m_buffer, G_SMALL_BUFFER, &m_bytesRead)); // full version
    
    GCStringOut response = bufferToGCStringOut(m_buffer, GALIL_BUFFER_SIZE); // get the response
    flushBuffer(); // flush the buffer
    
    return response;
    
} // GalilController::command

GCStringOut GalilController::motionComplete()
{
    return (GCStringOut) "1";  // TODO: implement
    
} // GalilController:: motionComplete

GReturn GalilController::motorsOn(bool axes[GALIL_NUM_AXES])
{
    bool any_on = false;
    std::string command = "SH ";
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        any_on = any_on || axes[i];
        if (axes[i])
            command += axisName(i);
        
    } // for
    
    if (any_on)
        this->command( command );
    
    return 0;
    
} // GalilController::motorsOn

GReturn GalilController::motorsOff(bool axes[GALIL_NUM_AXES])
{
    bool any_on = false;
    std::string command = "MO ";
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        any_on = any_on || axes[i];
        if (axes[i])
            command += axisName(i);
        
    } // for
    
    if (any_on)
        this->command( command );
    
    return 0;
    
} // GalilController::motorsOff

GReturn GalilController::moveAxes(long axes[GALIL_NUM_AXES], bool absolute)
{
    if (absolute)
        return moveAxesAbsolute(axes);
    
    else
        return moveAxesRelative(axes);
    
} // GalilController::moveAxes

GReturn GalilController::moveAxesAbsolute(long axes[GALIL_NUM_AXES])
{
    // format the command message
    std::string command = "PA " + commaSeparateValues(std::vector<long>(axes, axes + GALIL_NUM_AXES));
    std::string bg_command = "BG ";
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        //        command += std::to_string(axes[i]) + (i < GALIL_NUM_AXES - 1 ? ",": "");
        bg_command += axisName(i); // TODO: need to change to detect absolute changes
        
    } // for
    
    this->command(command); // send the move command
    this->command(bg_command); // send the begin command
    
    return 0;
    
} // GalilController::moveAxesAbsolute

GReturn GalilController::moveAxesRelative(long axes[GALIL_NUM_AXES])
{
    // format the command message
    std::string mv_command = "PR " + commaSeparateValues(std::vector<long>(axes, axes + GALIL_NUM_AXES));
    std::string bg_command = "BG ";
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        //        mv_command += std::to_string(axes[i]) + (i < GALIL_NUM_AXES - 1 ? ",": "");
        if (axes[i] != 0)
            bg_command += axisName(i);
        
    } // for
    
    this->command(mv_command); // send the move command
    this->command(bg_command); // send the begin command
    
    return 0;
    
} // GalilController::moveAxesRelative

long* GalilController::getPosition(bool axes[GALIL_NUM_AXES], bool absolute)
{
    // prepare the command
    std::string command = "";
    if (absolute)
        command += "PA ";
    else
        command += "PR ";
    
    for (int i = 0; i < GALIL_NUM_AXES; i++)
    {
        if (i < GALIL_NUM_AXES - 1 )
            command += axes[i] ? "?," : ",";
        
        else // end of list
            command += axes[i] ? "?" : "";
        
    } // for
    
    // get the response
    GCStringOut response = this->command(command);
    std::string s_response(response);
    
    // parse the response
    long* positions = new long[GALIL_NUM_AXES];
    for (int i = 0; i < GALIL_NUM_AXES; i++)
        positions[i] = NULL_LONG_AXIS; // initalize array

    /*
    char* token = strtok(response ,",");
    int counter = 0;
    while(token != NULL && counter < GALIL_NUM_AXES)
    {
        bool isnumeric = std::string(token).find_first_not_of("0123456789") == std::string::npos;
        std::cout << "Token = " << token << " | isnumeric = " << isnumeric << std::endl;
        if (isnumeric && axes[counter])
            positions[counter++] = std::stol(token);
        
        else
            counter++; // just increment the counter
        
        
        token = strtok(NULL, ",");

        std::cout << "positions[" << counter-1 << "] = " << positions[counter-1] << std::endl;
        
    } // while
    */
    std::string token;
    s_response += ","; // add-on to end to get last part
    std::size_t pos = 0;

    int counter = -1;
    auto update_counter = [&counter, &axes](){
        while(!axes[++counter] && counter < GALIL_NUM_AXES) 
            continue;
        };
    update_counter();
    
    while((pos = s_response.find(",")) != std::string::npos)
    {
        token = s_response.substr(0, pos);
        bool isnumeric = token.find_first_not_of("0123456789 ") == std::string::npos;

        if (isnumeric)
        {
            // add to result
            if (counter >= GALIL_NUM_AXES)
                break;
        

            positions[counter] = std::stol(token);
            update_counter();
            
        } // if

        s_response.erase(0, pos + 1); // remove the processed part of string

    } // while

    return positions;

    
    
} // GalilController::getPosition


GReturn GalilController::setPID_P(long kp_axes[GALIL_NUM_AXES])
{
    std::string command = "KP " + commaSeparateValues(std::vector<long>(kp_axes, kp_axes + GALIL_NUM_AXES));
    
    this->command(command);
    
    return 0;
    
} // GalilController::setPID_P


GReturn GalilController::setPID_I(long ki_axes[GALIL_NUM_AXES])
{
    std::string command = "KI " + commaSeparateValues(std::vector<long>(ki_axes, ki_axes + GALIL_NUM_AXES));
    
    this->command(command);
    
    return 0;
    
} // GalilController::setPID_I


GReturn GalilController::setPID_D(long kd_axes[GALIL_NUM_AXES])
{
    std::string command = "KD " + commaSeparateValues(std::vector<long>(kd_axes, kd_axes + GALIL_NUM_AXES));
    
    this->command(command);
    
    return 0;
    
} // GalilController::setPID_D


GReturn GalilController::setAcceleration(long ac_axes[GALIL_NUM_AXES])
{
    std::string command = "AC " + commaSeparateValues(std::vector<long>(ac_axes, ac_axes + GALIL_NUM_AXES));
    
    this->command(command); // send update command
    
    return 0;
    
} // GalilController::setAcceleration


GReturn GalilController::setDeceleration(long dc_axes[GALIL_NUM_AXES])
{
    std::string command = "DC " + commaSeparateValues(std::vector<long>(dc_axes, dc_axes + GALIL_NUM_AXES));
    
    this->command(command); // send update command
    
    return 0;
    
} // GalilController::setDeceleration


GReturn GalilController::setSpeed(long sp_axes[GALIL_NUM_AXES])
{
    std::string command = "SP " + commaSeparateValues(std::vector<long>(sp_axes, sp_axes + GALIL_NUM_AXES));
    
    this->command(command); // send update command
    
    return 0;
    
} // GalilController::setSpeed


GReturn GalilController::stopAxes(bool axes[GALIL_NUM_AXES])
{
    std::string command = "ST ";
    
    for (int i = 0; i < GALIL_NUM_AXES; i++)
        command += axes[i] ? std::string(1, axisName(i)) : "";
    
    this->command(command); // send the command
    
    return 0;
    
} // GalilController::stopAxes


GReturn GalilController::zeroAxes(bool axes[GALIL_NUM_AXES])
{
    std::string command = "DP ";
    
    for (int i = 0; i < GALIL_NUM_AXES; i++)
        command += axes[i] ? std::string(1, axisName(i)) : "";
    
    this->command(command); // send the command
    
    return 0;
    
} // GalilController::zeroAxes

