
import java.net.*;
import java.io.*;
import java.util.*;

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
         socketOut.write(s);
         socketOut.flush();   
   }
   
   public void close() throws IOException
   {
      conn.close();
   }
   
   public boolean connected()
   {
      return connected;
   }

   public void reconnect() throws UnknownHostException, IOException, SocketException
   {
      conn.close();
      conn = new Socket(host,port);
      conn.setSoTimeout(10000);
      connected = true;
   }

   //basic connection program
   public static void main(String[] args)
   {
      try
      {
         RpiConnection conn = new RpiConnection("192.168.17.1", 45000);
         Scanner scn = new Scanner(System.in);
         String input = "";
         System.out.println("HANDSHAKE FROM SERVER: " + conn.read());
         conn.write("HELLO FROM SERVER");
         do
         {
            try
            {
               System.out.println("RECEIVED FROM SERVER: " + conn.read());
               System.out.print("ENTER MSG TO SEND: ");
               input = scn.nextLine();
               System.out.println("");
            }
            catch(SocketTimeoutException ste)
            {
               conn.reconnect();
            }

         }while(input != "END" || conn.connected() == false);

      }
      catch(IOException ioe)
      {
         ioe.printStackTrace();
      }
   }

}