import java.awt.*;
import java.awt.event.*;

import javax.swing.*;

import java.io.*;


public class GamePanel extends JPanel implements Runnable/*, ActionListener*/{
	Thread gameThread;

	Parameters parameters;

	Color colorFondo = Color.BLACK;
	Color colorNumeros = Color.WHITE;
	Color colorWarning = Color.RED;
	
	
	public GamePanel()
	{
		this.setBackground(colorFondo);
		this.setDoubleBuffered(true);
		//parameters.setUp();
	}
	
	public void startGameThread() 
	{
		gameThread = new Thread(this);
		gameThread.start();
	}

	public void setParameters(Parameters parameters){
		this.parameters = parameters;
	}

	@Override
	public void run()
	{
		while(gameThread != null)
		{
			// 1. Update
			update();
	
			//2. Draw
			repaint();
		}
	}
	
	public void update()
	{

		/*try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		Timer timer = new Timer(1, this); //envía eventos cada milisegundo
		timer.start();*/
	}
	
	public void paintComponent(Graphics g)
	{
		super.paintComponent(g); 		
									 		
		Graphics2D g2 = (Graphics2D)g;

// formatos	
		Font font = new Font("Helvetica SemiBold", Font.BOLD, 370); //speed 
		g2.setFont(font);
		FontMetrics fm = g.getFontMetrics(); 
		Font font2 = new Font("Helvetica SemiBold", Font.BOLD, 70); //SoC
		g2.setFont(font2);
		FontMetrics fm2 = g.getFontMetrics(); 
		Font font3 = new Font("Helvetica SemiBold", Font.BOLD, 150); //batteries, tyres, inversor, engine 
		g2.setFont(font3);
		FontMetrics fm3 = g.getFontMetrics(); 
		Font font4 = new Font("Helvetica SemiBold", Font.BOLD, 80); //throttle, brake
		g2.setFont(font4);
		FontMetrics fm4 = g.getFontMetrics();
		Font font5 = new Font("Helvetica SemiBold", Font.BOLD, 50); //driving modes
		g2.setFont(font5);
		FontMetrics fm5 = g.getFontMetrics();  
		
		g.setColor(colorNumeros);
		// drawRect​(int x, int y, int width, int height, int arcWidth, int arcHeight)
// inversor
		g.setColor(colorNumeros);
		int x = 1650;
		int y = 150; 
		g2.setFont(font3);
		if(parameters.getT_Inversor()>65){
			g2.setColor(colorWarning);
		}
		g2.drawString(Integer.toString(parameters.getT_Inversor()), x, y);	

// batteries	
		g.setColor(colorNumeros);
		if(parameters.getT_Batteries()>50){
			g2.setColor(colorWarning);
		}	
		x = 1650;
		y = 305; 
		g2.setFont(font3);
		g2.drawString(Integer.toString(parameters.getT_Batteries()), x, y);	

// engine 	
		g.setColor(colorNumeros);
		if(parameters.getT_Engine()>75){
			g2.setColor(colorWarning);
		}
		x = 1650;
		y = 445; 
		g2.setFont(font3);
		g2.drawString(Integer.toString(parameters.getT_Engine()), x, y);	

// speed
		g.setColor(colorNumeros);
		if(parameters.getSpeed()>99){
			x = 550;
		}
		if(parameters.getSpeed()<100 && parameters.getSpeed()>9){
			x = 680;
		}
		if(parameters.getSpeed()<10){
			x = 810;
		}

		y = 380;//y = 360; 
		g2.setFont(font);
   	    g2.drawString(Integer.toString(parameters.getSpeed()), x, y);	

// SoC
		g.setColor(colorNumeros);
		x = 1450;
		y = 25;
		int width = 140;
		int height = 350;
		double socPercentage;
		socPercentage = (parameters.getSoc() * (height))/100;
		g2.fillRect(x, (y + height)-(int)socPercentage, width, (int)socPercentage);
		g2.setFont(font2);

		int textWidth = fm2.stringWidth(Integer.toString(parameters.getSoc())); 
		int textHeight = fm2.getHeight();
		int x_text = (width - textWidth) / 2 + x; 
		int y_text = y + height + 80;
		g2.drawString(Integer.toString(parameters.getSoc()), x_text, y_text);	

// brake 
		g.setColor(colorNumeros);
		x = 300;
		y = 25;
		width = 140;
		height = 350;			
		double brakePercentage;
		brakePercentage = (parameters.getBrake() * (height))/100;
		g2.fillRect(x, (y + height)-(int)brakePercentage, width, (int)brakePercentage);
		g2.setFont(font4);

		textWidth = fm4.stringWidth(Integer.toString(parameters.getBrake())); 
		textHeight = fm4.getHeight();
		x_text = (width - textWidth) / 2 + x; 
		y_text = y + height + 80;
		g2.drawString(Integer.toString(parameters.getBrake()), x_text, y_text);	

// throttle
		g.setColor(colorNumeros);
		x = 100;
		y = 25;
		width = 140;
		height = 350;		
		double throttlePercentage;
		throttlePercentage = (parameters.getThrottle() * (height))/100;
		g2.fillRect(x, (y + height)-(int)throttlePercentage, width, (int)throttlePercentage);
		g2.setFont(font4);

		textWidth = fm4.stringWidth(Integer.toString(parameters.getThrottle())); 
		textHeight = fm4.getHeight();
		x_text = (width - textWidth) / 2 + x; 
		y_text = y + height + 80;
		g2.drawString(Integer.toString(parameters.getThrottle()), x_text, y_text);	
	}

//evento
	/*public void actionPerformed(ActionEvent e) {
		//while(true){
		//parameters.readCAN();
		//}
	}*/

}
