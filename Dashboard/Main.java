import java.awt.*;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class Main {
	public static void main(String argv[]) {
	
		JFrame window = new JFrame();
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		//window.setUndecorated(false);
		Parameters parameters = new Parameters();
		GamePanel gamePanel = new GamePanel();
		gamePanel.setParameters(parameters);
		window.add(gamePanel);

		//window.pack();
		
		gamePanel.startGameThread();

        	// Set the size and position of the window
        	GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
        	GraphicsDevice gd = ge.getDefaultScreenDevice();
        	DisplayMode dm = gd.getDisplayMode();
        	int screenWidth = dm.getWidth();
        	int screenHeight = dm.getHeight();
        	int windowWidth = 1920;
        	int windowHeight = 480;
        	int cornerX = 0; // X-coordinate of the corner (left side of the screen)
        	int cornerY = 0; // Y-coordinate of the corner (bottom of the screen)

        	window.setSize(windowWidth, windowHeight);
        	window.setLocation(cornerX, cornerY);

        	window.setVisible(true);

		try {
		    Thread.sleep(2000);
		    parameters.setUp();
		    Thread.sleep(1000);
		    parameters.start();
		} catch(Exception e){
		}
	}
}
