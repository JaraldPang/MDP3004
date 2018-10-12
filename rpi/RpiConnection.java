import java.net.*;
import java.io.*;
import java.util.*;
import java.util.Random;

public class RpiConnection
{
   private Socket conn;
   private PrintWriter socketOut;
   private BufferedReader socketIn;
   private String host;
   private int port;
   private boolean connected = false;

   public RpiConnection(String host, int port) throws IOException
   {
      this.host = host;
      this.port = port;
      this.conn = new Socket(host,port);
      this.connected = true;
      this.socketOut = new PrintWriter(conn.getOutputStream());
      this.socketIn = new BufferedReader(new InputStreamReader(conn.getInputStream()));
   }
   
   public String read() throws SocketTimeoutException, IOException
   {
      String s = socketIn.readLine();
      if(s == null)
      {
         connected = false;
         throw new SocketTimeoutException("Read returned null. Server has closed connection due to timeout");
      }
      else
      {
         return s;
      }
   }
   
   public void write(String s)
   {
      socketOut.write(s + "\n");
      socketOut.flush();   
   }
   
   public void writeToArduino(String s)
   {
      socketOut.write("AR" + s);
      socketOut.flush();   
   }
   
   public void writeToAndroid(String s)
   {      
      socketOut.write("AN" + s);
      socketOut.flush();      
   }
   
   public void close() throws IOException
   {
      conn.close();
   }
   
   public boolean getConnected()
   {
      return connected;
   }
   
   public void setConnected(boolean connect)
   {
      this.connected = connect;
   }

   public void reconnect() throws IOException
   {
      connected = false;
      while(true)
      {
         try
         {
            System.out.println("Reconnecting...");
            Thread.sleep(1000);
            conn.close();
            conn = new Socket(host,port);
            conn.setSoTimeout(10000);
            this.socketOut = new PrintWriter(conn.getOutputStream());
            this.socketIn = new BufferedReader(new InputStreamReader(conn.getInputStream()));
            connected = true;
            break;
         }
         catch(SocketTimeoutException ste_reconnect){
            continue;}            
         catch(UnknownHostException uhe){ 
            continue; }
         catch(NoRouteToHostException nrthe){ 
            continue;}
         catch(SocketException se){ 
            continue; }
         catch(InterruptedException ie){ 
            continue; }
      }      
   }

   //basic connection program
   public static void main(String[] args)
   {
      emulateAlgo2();
   }
   
   public static void emulateAlgo1()
   {
      try
      {
         Random ran = new Random();
         RpiConnection conn = new RpiConnection("192.168.17.1", 45000);
         String msg;
         for(int i = 0; i < 5; i++)
         {
            switch(ran.nextInt(4))
            {
               case 0:
                  conn.write("arw");
                  System.out.println("Forward");
                  break;
               case 1:
                  conn.write("ara");
                  System.out.println("Turn Left");
                  break;
               case 2:
                  conn.write("ars");
                  System.out.println("Backwards");
                  break;
               case 3:
                  conn.write("ard");
                  System.out.println("Turn Right");
                  break;                  
            }
            //emulate ack from algo
            do
            {            
               msg = conn.read();
            }while(msg != "ok");
            
            conn.write("rpic"+i);
         }
         conn.close();
      }
      catch(IOException ioe)
      {
         ioe.printStackTrace();
      }
   }
   
   public static void emulateAlgo2()
   {
      try
      {
         Random ran = new Random();
         RpiConnection conn = new RpiConnection("192.168.17.1", 45000);
         String msg;

         conn.write("arw");
         System.out.println("Forward");
         do
         {            
            msg = conn.read();
            System.out.println("Received:" + msg);
         }while(!msg.equalsIgnoreCase("ok"));
         conn.write("rpi1,2,u");
         
         conn.write("ara");
         System.out.println("Turn Left");
         do
         {            
            msg = conn.read();
            System.out.println("Received:" + msg);
         }while(!msg.equalsIgnoreCase("ok"));
         conn.write("rpi1,2,l");
         
         conn.write("ard");
         System.out.println("Turn Right");
         do
         {            
            msg = conn.read();
            System.out.println("Received:" + msg);
         }while(!msg.equalsIgnoreCase("ok"));
         conn.write("rpi1,2,u");
         
         conn.write("ars");
         System.out.println("Backwards");
         do
         {            
            msg = conn.read();
            System.out.println("Received:" + msg);
         }while(!msg.equalsIgnoreCase("ok"));
         conn.write("rpi1,1,u");
         
         try
         {
            Thread.sleep(1000);
         }
         catch(InterruptedException ie){}
         
         conn.write("rpi1,1,u");
         conn.close();
      }
      catch(IOException ioe)
      {
         ioe.printStackTrace();
      }
   }   

   public static void provideInput()
   {
      try
      {
         RpiConnection conn = new RpiConnection("192.168.17.1", 45000);
         Scanner scn = new Scanner(System.in);
         String input = "";
         do
         {
            try
            {
              System.out.print("ENTER MSG TO SEND: ");
              input = scn.nextLine();
              System.out.println("");
              conn.write(input);
              System.out.println("RECEIVED: " + conn.read());
            }
            catch(SocketTimeoutException ste)
            {
               conn.reconnect();
            }
         
         }while(input != "END" || conn.getConnected() == false);
         conn.close();
      }
      catch(IOException ioe)
      {
         ioe.printStackTrace();
      }
   }

}