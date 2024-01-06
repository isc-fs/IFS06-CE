import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import java.util.Random;
public class Parameters extends Thread{
    int soc;
    int speed;
    int throttle;
    int brake;
    int t_engine;
    int t_inversor;
    int t_batteries;

    String id;
    String[] values;
    String comando = "";
    Process proceso;
    BufferedReader reader;
    public int getSoc(){ 
        return soc;
    }
    public int getSpeed(){ 
        return speed;
    }
    public int getThrottle(){ 
        return throttle;
    }
    public int getBrake(){ 
        return brake;//random.nextInt(10);
    }
    public int getT_Engine(){ 
        return t_engine;
    }
    public int getT_Inversor(){ 
        return t_inversor;
    }
    public int getT_Batteries(){ 
        return t_batteries;
    } 

   public void setUp(){
	try{	
            // Comando que quieres ejecutar
            comando = "candump can0";

            // Ejecuta el comando en el terminal
            proceso = Runtime.getRuntime().exec(comando);
	    reader = new BufferedReader(new InputStreamReader(proceso.getInputStream()));
        } catch(Exception e){
	} 
 }

    public void run(){
  	try{
            // Captura la salida del proceso
            String linea;
            //linea = reader.readLine();

            while((linea = reader.readLine()) != null) {
                values = linea.split("\\s+");
                id = values[2];
                int number  = Integer.parseInt(values[4]+values[5],16);
                String value = Integer.toString(number);
                if(id.equals("304")){ //Speed
                    //this.speed = (((Integer.parseInt(values[4]+values[5])* 2 * 3.1416) / 60) / 3);
                }
                else if(id.equals("106")){ //Throttle
                    this.throttle = Integer.parseInt(value);
                }
                else if(id.equals("103")){ //Brake
                    this.brake = Integer.parseInt(value);
                }
                else if(id.equals("12C")){ //SoC
                    this.soc = (Integer.parseInt(value) - 2800) / 14;
                }
                else if(id.equals("301")){ //TEng
                    this.t_engine = (Integer.parseInt(value) - 9452) / 52;
                }
                else if(id.equals("302")){ //TInv
                    this.t_inversor = (Integer.parseInt(value) - 17621) / 84;
                }
                else if(id.equals("400")){ //TBat
                    this.t_batteries = Integer.parseInt(value);
                }

            }

            // Espera a que el proceso termine
            //proceso.waitFor();

            // Cierra el lector
            //reader.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

}
