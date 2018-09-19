
import java.net.*;
import java.io.*;
import java.util.*;

public class RpiConnection
{

   public static void main(String[] args)
   {
      /* TODO: RELOCATE VARS TO INSTANCE LEVEL */ 
      try
      {
         Socket conn = new Socket("192.168.1.28", 45000);
         PrintWriter socketOut = new PrintWriter(conn.getOutputStream());
			BufferedReader socketIn = new BufferedReader(new InputStreamReader(conn.getInputStream()));
         Scanner scn = new Scanner(System.in);
         String input;
         socketOut.write("HELLO FROM CLIENT!");
         socketOut.flush();
         System.out.println("HANDSHAKE FROM SERVER: " + socketIn.readLine());
         do
         {
            System.out.println("RECEIVED FROM SERVER: " + socketIn.readLine());
            System.out.print("ENTER MSG TO SEND: ");
            input = scn.nextLine();
            socketOut.write(input);
            socketOut.flush();
            System.out.println("");
         }while(input != "END");
      
         conn.close();
      }
		catch(IOException ioe)
		{
			ioe.printStackTrace();
		}
      
   }

}