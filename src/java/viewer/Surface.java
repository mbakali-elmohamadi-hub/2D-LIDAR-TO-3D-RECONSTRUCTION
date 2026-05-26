/*
 * Java Swing 3D Point Cloud Viewer and Scan Launcher
 *
 * This GUI allows the user to start a LiDAR scan, launch the C++ acquisition
 * executable, run the Python outlier-filtering script, and visualize the
 * resulting 3D point cloud. It also supports loading an existing point file
 * in x:<value>,y:<value>,z:<value> format.
 */

import javax.swing.*;  
import javax.swing.border.LineBorder;
import javax.swing.plaf.basic.BasicSliderUI;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import com.fazecast.jSerialComm.SerialPort;

public class Surface extends JPanel implements MouseMotionListener, MouseWheelListener, MouseListener {

    static class Point3D {
        double x, y, z;
        Point3D(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    }

    java.util.List<Point3D> points = new ArrayList<>();
    double angleX = 0, angleY = 0, angleZ = 0;
    double zoom = 1;
    int lastX = -1, lastY = -1;
    double minX, maxX, minY, maxY, minZ, maxZ;
    Point3D nearestPoint = null;

    JLabel coordLabel;

    public Surface() {
        addMouseMotionListener(this);
        addMouseWheelListener(this);
        addMouseListener(this);
    }

    public void loadFile(String filename) {
        points.clear();
        minX = minY = minZ = Double.POSITIVE_INFINITY;
        maxX = maxY = maxZ = Double.NEGATIVE_INFINITY;

        try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
            String line;
            while ((line = br.readLine()) != null) {
                try {
                    String[] parts = line.split(",");
                    double x = 0, y = 0, z = 0;
                    for (String part : parts) {
                        String[] kv = part.split(":");
                        if (kv.length == 2) {
                            double val = Double.parseDouble(kv[1]);
                            if (kv[0].contains("x")) x = val;
                            else if (kv[0].contains("y")) y = val;
                            else if (kv[0].contains("z")) z = val;
                        }
                    }
                    y = -y;
                    points.add(new Point3D(x, y, z));
                    minX = Math.min(minX, x);
                    maxX = Math.max(maxX, x);
                    minY = Math.min(minY, y);
                    maxY = Math.max(maxY, y);
                    minZ = Math.min(minZ, z);
                    maxZ = Math.max(maxZ, z);
                } catch (Exception ignored) {}
            }
        } catch (IOException e) {
            JOptionPane.showMessageDialog(this, "Error lecture file : " + e.getMessage(),
                    "Error", JOptionPane.ERROR_MESSAGE);
        }
        repaint();
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (points.isEmpty()) return;

        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.WHITE);
        g2d.fillRect(0, 0, getWidth(), getHeight());

        double[][] rotX = {
                {1, 0, 0},
                {0, Math.cos(angleX), -Math.sin(angleX)},
                {0, Math.sin(angleX), Math.cos(angleX)}
        };

        double[][] rotY = {
                {Math.cos(angleY), 0, Math.sin(angleY)},
                {0, 1, 0},
                {-Math.sin(angleY), 0, Math.cos(angleY)}
        };

        double[][] rotZ = {
                {Math.cos(angleZ), -Math.sin(angleZ), 0},
                {Math.sin(angleZ), Math.cos(angleZ), 0},
                {0, 0, 1}
        };

        for (Point3D p : points) {
            double[] v = {p.x, p.y, p.z};
            v = mult(rotZ, v);
            v = mult(rotY, v);
            v = mult(rotX, v);

            int sx = (int)(getWidth()/2 + v[0]*zoom);
            int sy = (int)(getHeight()/2 - v[1]*zoom);

            float normZ = (float)((p.z - minZ) / (maxZ - minZ + 1e-8));
            Color c = Color.getHSBColor(normZ * 0.7f, 1f, 1f);
            g2d.setColor(c);
            g2d.drawLine(sx, sy, sx, sy);
        }

        drawAxis(g2d, rotX, rotY, rotZ);
    }

    double[] mult(double[][] m, double[] v) {
        return new double[]{
                m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
                m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
                m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]
        };
    }

    void drawAxis(Graphics2D g2d, double[][] rotX, double[][] rotY, double[][] rotZ) {
        int ox = getWidth() / 2;
        int oy = getHeight() / 2;

        String[] labels = {"X", "Y", "Z"};
        Color[] colors = {Color.RED, Color.GREEN.darker(), Color.BLUE};
        double[][] directions = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        double fixedLength = 100;

        for (int i = 0; i < 3; i++) {
            double[] dir = directions[i];
            double[] v = mult(rotZ, new double[]{dir[0] * fixedLength, dir[1] * fixedLength, dir[2] * fixedLength});
            v = mult(rotY, v);
            v = mult(rotX, v);

            int ex = (int)(ox + v[0]*zoom);
            int ey = (int)(oy - v[1]*zoom);

            g2d.setColor(colors[i]);
            g2d.setStroke(new BasicStroke(2));
            g2d.drawLine(ox, oy, ex, ey);
            g2d.drawString(labels[i], ex + 5, ey);
        }
    }

    private void findNearestPoint(int mouseX, int mouseY) {
        double distMin = Double.MAX_VALUE;
        nearestPoint = null;

        double[][] rotX = {
                {1, 0, 0},
                {0, Math.cos(angleX), -Math.sin(angleX)},
                {0, Math.sin(angleX), Math.cos(angleX)}
        };
        double[][] rotY = {
                {Math.cos(angleY), 0, Math.sin(angleY)},
                {0, 1, 0},
                {-Math.sin(angleY), 0, Math.cos(angleY)}
        };
        double[][] rotZ = {
                {Math.cos(angleZ), -Math.sin(angleZ), 0},
                {Math.sin(angleZ), Math.cos(angleZ), 0},
                {0, 0, 1}
        };

        for (Point3D p : points) {
            double[] v = {p.x, p.y, p.z};
            v = mult(rotZ, v);
            v = mult(rotY, v);
            v = mult(rotX, v);

            int sx = (int)(getWidth()/2 + v[0]*zoom);
            int sy = (int)(getHeight()/2 - v[1]*zoom);
            double dist = Math.hypot(sx - mouseX, sy - mouseY);
            if (dist < distMin && dist < 15) {
                distMin = dist;
                nearestPoint = p;
            }
        }

        if (coordLabel != null) {
            if (nearestPoint != null) {
                coordLabel.setText(String.format("Nearest coordinates : x=%.2f, y=%.2f, z=%.2f", nearestPoint.x, nearestPoint.y, nearestPoint.z));
            } else {
                coordLabel.setText("No nearby point under the mouse (rotation and zoom are available with the mouse)");
            }
        }
    }

    public void mouseDragged(MouseEvent e) {
        if (lastX >= 0 && lastY >= 0) {
            int dx = e.getX() - lastX;
            int dy = e.getY() - lastY;
            if ((e.getModifiersEx() & MouseEvent.BUTTON1_DOWN_MASK) != 0) {
                if ((e.getModifiersEx() & MouseEvent.SHIFT_DOWN_MASK) != 0) {
                    angleZ += dx * 0.01;
                } else {
                    angleY += dx * 0.01;
                    angleX += dy * 0.01;
                }
                repaint();
            }
        }
        lastX = e.getX();
        lastY = e.getY();
    }

    public void mouseMoved(MouseEvent e) {
        findNearestPoint(e.getX(), e.getY());
        repaint();
    }

    public void mousePressed(MouseEvent e) { lastX = e.getX(); lastY = e.getY(); }
    public void mouseReleased(MouseEvent e) { lastX = -1; lastY = -1; }
    public void mouseClicked(MouseEvent e) {}
    public void mouseEntered(MouseEvent e) {}
    public void mouseExited(MouseEvent e) {}

    public void mouseWheelMoved(MouseWheelEvent e) {
        zoom *= (1 - e.getWheelRotation() * 0.1);
        zoom = Math.max(0.1, Math.min(zoom, 10));
        repaint();
    }

    static void setSliderStyle(JSlider slider) {
        slider.setUI(new BasicSliderUI(slider) {
            @Override
            public void paintTrack(Graphics g) {
                Graphics2D g2 = (Graphics2D) g;
                g2.setColor(Color.LIGHT_GRAY);
                g2.fillRect(trackRect.x, trackRect.y + trackRect.height / 2 - 2, trackRect.width, 4);
            }

            @Override
            public void paintThumb(Graphics g) {
                Graphics2D g2 = (Graphics2D) g.create();
                g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
                g2.setColor(Color.WHITE);
                int w = 12, h = 16;
                int x = thumbRect.x + (thumbRect.width - w) / 2;
                int y = thumbRect.y + (thumbRect.height - h) / 2;
                g2.fillRoundRect(x, y, w, h, 4, 4);
                g2.dispose();
            }
        });
    }

    static class BackgroundImagePanel extends JPanel {
        private Image backgroundImage;
        public BackgroundImagePanel(Image img) {
            this.backgroundImage = img;
            setLayout(new GridBagLayout());
        }
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            if (backgroundImage != null) {
                g.drawImage(backgroundImage, 0, 0, getWidth(), getHeight(), this);
            }
        }
    }

    public static void main(String[] args) {
        final JFrame frame = new JFrame("3D Point Cloud Viewer");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1200, 700);
        frame.setLayout(new BorderLayout());

        Surface surface3D = new Surface();

        JLabel coordLabel = new JLabel("No nearby point under the mouse (rotation and zoom are available with the mouse)", SwingConstants.CENTER);
        coordLabel.setOpaque(true);
        coordLabel.setBackground(Color.DARK_GRAY);
        coordLabel.setForeground(Color.WHITE);
        coordLabel.setBorder(new LineBorder(Color.GRAY));
        coordLabel.setPreferredSize(new Dimension(900, 30));
        coordLabel.setVisible(false);
        surface3D.coordLabel = coordLabel;

        JButton zoomIn = new JButton("Zoom +");
        JButton zoomOut = new JButton("Zoom -");
        JButton backButton = new JButton("Back");

        JButton[] allButtons = {zoomIn, zoomOut, backButton};
        for (JButton b : allButtons) {
            b.setBackground(Color.DARK_GRAY);
            b.setForeground(Color.WHITE);
            b.setFont(new Font("Arial", Font.BOLD, 13));
            b.setFocusPainted(false);
            b.setPreferredSize(new Dimension(80, 24));
            b.setMinimumSize(new Dimension(80, 24));
            b.setMaximumSize(new Dimension(80, 24));
            b.setBorder(BorderFactory.createLineBorder(Color.WHITE));
        }

        JPanel slidersPanel = new JPanel(new GridLayout(3, 1, 5, 5));
        slidersPanel.setBackground(Color.DARK_GRAY);

        JSlider sliderX = new JSlider(-360, 360, 0);
        JSlider sliderY = new JSlider(-360, 360, 0);
        JSlider sliderZ = new JSlider(-360, 360, 0);
        JLabel labelX = new JLabel("Rotation X: 0°");
        JLabel labelY = new JLabel("Rotation Y: 0°");
        JLabel labelZ = new JLabel("Rotation Z: 0°");

        JLabel[] labels = {labelX, labelY, labelZ};
        JSlider[] sliders = {sliderX, sliderY, sliderZ};

        for (int i = 0; i < 3; i++) {
            labels[i].setForeground(Color.WHITE);
            sliders[i].setPaintTicks(false);
            sliders[i].setBackground(Color.DARK_GRAY);
            sliders[i].setForeground(Color.WHITE);
            setSliderStyle(sliders[i]);
            JPanel p = new JPanel(new FlowLayout(FlowLayout.LEFT));
            p.setBackground(Color.DARK_GRAY);
            p.add(labels[i]);
            p.add(sliders[i]);
            slidersPanel.add(p);
        }

        JPanel buttonColumnPanel = new JPanel();
        buttonColumnPanel.setBackground(Color.DARK_GRAY);
        buttonColumnPanel.setLayout(new BoxLayout(buttonColumnPanel, BoxLayout.Y_AXIS));
        buttonColumnPanel.add(Box.createVerticalGlue());
        buttonColumnPanel.add(zoomIn);
        buttonColumnPanel.add(Box.createRigidArea(new Dimension(0, 10)));
        buttonColumnPanel.add(zoomOut);
        buttonColumnPanel.add(Box.createRigidArea(new Dimension(0, 10)));
        buttonColumnPanel.add(backButton);
        buttonColumnPanel.add(Box.createVerticalGlue());

        JPanel leftMarginPanel = new JPanel(new BorderLayout());
        leftMarginPanel.setBackground(Color.DARK_GRAY);
        leftMarginPanel.add(Box.createRigidArea(new Dimension(10, 0)), BorderLayout.WEST);
        leftMarginPanel.add(buttonColumnPanel, BorderLayout.CENTER);

        JPanel verticalCenterPanel = new JPanel(new BorderLayout());
        verticalCenterPanel.setBackground(Color.DARK_GRAY);
        verticalCenterPanel.add(leftMarginPanel, BorderLayout.CENTER);

        JPanel buttonsPanel = new JPanel(new BorderLayout());
        buttonsPanel.setBackground(Color.DARK_GRAY);
        buttonsPanel.add(verticalCenterPanel, BorderLayout.WEST);
        buttonsPanel.add(slidersPanel, BorderLayout.EAST);
        buttonsPanel.setVisible(false);

        JPanel bottomPanel = new JPanel(new BorderLayout());
        bottomPanel.add(coordLabel, BorderLayout.NORTH);
        bottomPanel.add(buttonsPanel, BorderLayout.CENTER);

        Image imgFond = null;
        try {
            imgFond = javax.imageio.ImageIO.read(new File("src/java/viewer/surface.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        JPanel homeBackgroundPanel = new BackgroundImagePanel(imgFond);
        homeBackgroundPanel.setLayout(new BoxLayout(homeBackgroundPanel, BoxLayout.Y_AXIS));

        JPanel titleButtonPanel = new JPanel();
        titleButtonPanel.setLayout(new BoxLayout(titleButtonPanel, BoxLayout.Y_AXIS));
        titleButtonPanel.setOpaque(false);

        JLabel titleLabel = new JLabel("3D Point Cloud Visualization", SwingConstants.CENTER);
        titleLabel.setFont(new Font("Serif", Font.BOLD, 17));
        titleLabel.setAlignmentX(Component.CENTER_ALIGNMENT);
        titleLabel.setForeground(Color.WHITE);

        JLabel warningLabel = new JLabel("Please connect the Arduino and LiDAR COM ports before continuing", SwingConstants.CENTER);
        warningLabel.setFont(new Font("Serif", Font.BOLD, 15));
        warningLabel.setAlignmentX(Component.CENTER_ALIGNMENT);
        warningLabel.setForeground(Color.WHITE);

        JLabel messageLabel = new JLabel("Press to start the LiDAR scan and display the reconstructed points", SwingConstants.CENTER);
        messageLabel.setFont(new Font("Serif", Font.BOLD, 15));
        messageLabel.setAlignmentX(Component.CENTER_ALIGNMENT);
        messageLabel.setForeground(Color.WHITE);

        JLabel zInfoLabel = new JLabel("<html>The scan uses a maximum Z height of <b>407 mm</b>.<br>Please choose an <b>integer value below 408</b>.</html>", SwingConstants.CENTER);
        zInfoLabel.setFont(new Font("Serif", Font.PLAIN, 15));
        zInfoLabel.setAlignmentX(Component.CENTER_ALIGNMENT);
        zInfoLabel.setForeground(Color.WHITE);


        // On ne met pas les combos ports ni le champ distance dans l'accueil (ils seront dans le dialog)

        JButton startScanButton = new JButton("Start le Scan");
        startScanButton.setFont(new Font("Arial", Font.BOLD, 12));
        startScanButton.setBackground(Color.DARK_GRAY);
        startScanButton.setForeground(Color.WHITE);
        startScanButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        startScanButton.setFocusPainted(false);
        startScanButton.setBorder(BorderFactory.createLineBorder(Color.WHITE));
        startScanButton.setMaximumSize(new Dimension(150, 28));
        startScanButton.setPreferredSize(new Dimension(150, 28));

        JPanel cards = new JPanel(new CardLayout());
        cards.add(homeBackgroundPanel, "HOME");
        cards.add(surface3D, "SURFACE3D");
        frame.add(cards, BorderLayout.CENTER);

        CardLayout cl = (CardLayout)(cards.getLayout());
        cl.show(cards, "HOME");

        JLabel loadFileExplanation = new JLabel("Load an existing file containing 3D points to visualize", SwingConstants.CENTER);
        loadFileExplanation.setFont(new Font("Serif", Font.BOLD, 15));
        loadFileExplanation.setAlignmentX(Component.CENTER_ALIGNMENT);
        loadFileExplanation.setForeground(Color.WHITE);

        JButton loadFileButton = new JButton("Charger un file");
        loadFileButton.setFont(new Font("Arial", Font.BOLD, 12));
        loadFileButton.setBackground(Color.DARK_GRAY);
        loadFileButton.setForeground(Color.WHITE);
        loadFileButton.setAlignmentX(Component.CENTER_ALIGNMENT);
        loadFileButton.setFocusPainted(false);
        loadFileButton.setBorder(BorderFactory.createLineBorder(Color.WHITE));
        loadFileButton.setMaximumSize(new Dimension(150, 28));
        loadFileButton.setPreferredSize(new Dimension(150, 28));

        // Add components to the home panel.
        titleButtonPanel.add(Box.createRigidArea(new Dimension(0, 20)));
        titleButtonPanel.add(titleLabel);
        titleButtonPanel.add(Box.createRigidArea(new Dimension(0, 3)));
        titleButtonPanel.add(warningLabel);
        titleButtonPanel.add(Box.createRigidArea(new Dimension(0, 3)));
        titleButtonPanel.add(messageLabel);
        titleButtonPanel.add(Box.createRigidArea(new Dimension(0, 3)));  // un petit espace avant le message zInfoLabel
        titleButtonPanel.add(zInfoLabel);
        titleButtonPanel.add(Box.createRigidArea(new Dimension(0, 5)));
        titleButtonPanel.add(startScanButton);
        titleButtonPanel.add(Box.createRigidArea(new Dimension(0, 10)));
        titleButtonPanel.add(loadFileExplanation);
        titleButtonPanel.add(Box.createRigidArea(new Dimension(0, 5)));
        titleButtonPanel.add(loadFileButton);
        titleButtonPanel.add(Box.createRigidArea(new Dimension(0, 10)));

        homeBackgroundPanel.add(Box.createVerticalGlue());
        homeBackgroundPanel.add(titleButtonPanel);
        homeBackgroundPanel.add(Box.createVerticalGlue());

        bottomPanel.add(buttonsPanel, BorderLayout.SOUTH); 
        frame.add(bottomPanel, BorderLayout.SOUTH);

        // --------- Action du button Start le Scan (ouvre un JDialog) ---------
        startScanButton.addActionListener(e -> {
            // Create the modal scan-parameter dialog.
            JDialog dialog = new JDialog(frame, "LiDAR Scan Parameters", true);
            dialog.setSize(400, 220);
            dialog.setResizable(false);
            dialog.setLocationRelativeTo(frame);
            dialog.setLayout(new GridBagLayout());
            GridBagConstraints gbc = new GridBagConstraints();
            gbc.insets = new Insets(5,5,5,5);
            gbc.fill = GridBagConstraints.HORIZONTAL;

            // Label and combo box for the LiDAR port.
            JLabel labelPortLiDAR = new JLabel("Port COM LIDAR:");
            gbc.gridx = 0; gbc.gridy = 0; gbc.weightx = 0.3;
            dialog.add(labelPortLiDAR, gbc);

            JComboBox<String> comboPortLiDAR = new JComboBox<>();
            for (SerialPort sp : SerialPort.getCommPorts()) {
                comboPortLiDAR.addItem(sp.getSystemPortName());
            }
            gbc.gridx = 1; gbc.gridy = 0; gbc.weightx = 0.7;
            dialog.add(comboPortLiDAR, gbc);

            // Label and combo box for the Arduino port.
            JLabel labelPortArduino = new JLabel("Port COM Arduino:");
            gbc.gridx = 0; gbc.gridy = 1; gbc.weightx = 0.3;
            dialog.add(labelPortArduino, gbc);

            JComboBox<String> comboPortArduino = new JComboBox<>();
            for (SerialPort sp : SerialPort.getCommPorts()) {
                comboPortArduino.addItem(sp.getSystemPortName());
            }
            gbc.gridx = 1; gbc.gridy = 1; gbc.weightx = 0.7;
            dialog.add(comboPortArduino, gbc);

            // Label and text field for the scan height.
            JLabel labelDistance = new JLabel("Height (mm):");
            gbc.gridx = 0; gbc.gridy = 2; gbc.weightx = 0.3;
            dialog.add(labelDistance, gbc);

            JTextField fieldDistance = new JTextField("");
            gbc.gridx = 1; gbc.gridy = 2; gbc.weightx = 0.7;
            dialog.add(fieldDistance, gbc);

            // Button Start
            JButton startButton = new JButton("Start");
            gbc.gridx = 0; gbc.gridy = 3; gbc.gridwidth = 2; gbc.weightx = 1.0;
            dialog.add(startButton, gbc);

            startButton.addActionListener(ev -> {
            
                String portLiDAR = (String) comboPortLiDAR.getSelectedItem();
                String portArduino = (String) comboPortArduino.getSelectedItem();
                String distanceStr = fieldDistance.getText().trim();

                int dist;
try {
    if (distanceStr.contains(".") || distanceStr.contains(",")) {
        throw new NumberFormatException(); // On rejette les valeurs avec virgule ou point
    }
    dist = Integer.parseInt(distanceStr);
    if (dist <= 0 || dist >= 408) {
        JOptionPane.showMessageDialog(dialog,
                "<html> The height must be an integer <b>greater than 0</b> and <b>less than 408 mm</b>.<br>Decimal values are not allowed.</html>",
                "Invalid distance",
                JOptionPane.WARNING_MESSAGE);
        return;
    }
} catch (NumberFormatException ex) {
    JOptionPane.showMessageDialog(dialog,
            "<html> The height must be an <b>integer without decimal separators</b>.<br>Please correct the entered value.</html>",
            "Error de saisie",
            JOptionPane.ERROR_MESSAGE);
    return;
}


                if (portLiDAR == null || portArduino == null || distanceStr.isEmpty()) {
                    JOptionPane.showMessageDialog(dialog, "Please fill in all fields", "Error", JOptionPane.ERROR_MESSAGE);
                    return;
                }

                if (portLiDAR.equals(portArduino)) {
                    JOptionPane.showMessageDialog(dialog,
                    "The LiDAR COM port and Arduino COM port must be different.",
                    "Selection error",
                    JOptionPane.ERROR_MESSAGE);
                    return; // Stop here and do not launch the scan.
                }

                try {
                    dist = Integer.parseInt(distanceStr);
                } catch (NumberFormatException ex) {
                    JOptionPane.showMessageDialog(dialog, "Invalid distance", "Error", JOptionPane.ERROR_MESSAGE);
                    return;
                }

                // Start ultra_simple.exe avec arguments : portLiDAR portArduino distance
                    // Temporary scan-progress window.

                
    JDialog scanDialog = new JDialog(frame, "Scan start", false);
    scanDialog.setSize(250, 100);
    scanDialog.setLocationRelativeTo(frame);
    JLabel labelWait = new JLabel("<html>Please press the physical push button.<br>The scan is starting.</html>", SwingConstants.CENTER);
    scanDialog.add(labelWait);
    scanDialog.setDefaultCloseOperation(JDialog.DISPOSE_ON_CLOSE);

    // Close the parameter dialog.
    dialog.dispose();

    // Display the scan-progress dialog.
    scanDialog.setVisible(true);


    try {
        String cmd = String.format("C:\\Users\\HP\\Documents\\projet\\lidar\\rplidar_sdk-master\\output\\Win32\\Release\\ultra_simple.exe --channel --serial %s 115200 %s %d", portLiDAR, portArduino, dist);
        Process proc = Runtime.getRuntime().exec(cmd);

        BufferedReader stdInput = new BufferedReader(new InputStreamReader(proc.getInputStream()));
        BufferedReader stdError = new BufferedReader(new InputStreamReader(proc.getErrorStream()));

        new Thread(() -> {
            String s;
            try {
                while ((s = stdInput.readLine()) != null) System.out.println(s);
            } catch (IOException ignored) {}
        }).start();

        new Thread(() -> {
            String s;
            try {
                while ((s = stdError.readLine()) != null) System.err.println(s);
            } catch (IOException ignored) {}
        }).start();

        new Thread(() -> {
    try {
        proc.waitFor();
        SwingUtilities.invokeLater(() -> {
            scanDialog.dispose();

            // Check whether the output file contains scanned data.
            String resultsFilePath = "C:\\Users\\HP\\Documents\\projet\\resultats.txt";

            File file = new File(resultsFilePath);
            boolean isFileEmpty = true;

            if (file.exists()) {
                try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
                    if (reader.readLine() != null) {
                        isFileEmpty = false; // il y a au moins une line
                    }
                } catch (IOException exRead) {
                    exRead.printStackTrace();
                }
            }

            if (isFileEmpty) {
                JOptionPane.showMessageDialog(frame,
                        "<html><b>Error: no scanned data.</b><br>Check COM ports or cable connections.<br><br>The application will now close.</html>",
                        "Scan failed",
                        JOptionPane.ERROR_MESSAGE);
                        System.exit(0);
                return;
            }


            JDialog reconstructionDialog = new JDialog(frame, "3D Reconstruction", false);
            reconstructionDialog.setSize(250, 100);
            reconstructionDialog.setLocationRelativeTo(frame);
            JLabel labelRecon = new JLabel("<html>3D Reconstruction en cours...<br>Veuillez patienter.</html>", SwingConstants.CENTER);
            reconstructionDialog.add(labelRecon);
            reconstructionDialog.setDefaultCloseOperation(JDialog.DISPOSE_ON_CLOSE);
            reconstructionDialog.setVisible(true);

        // Path of the Python filtering script.
            String pythonScript = "C:\\Users\\HP\\Documents\\projet\\python\\filtrage_lof.py";
            resultsFilePath = "C:\\Users\\HP\\Documents\\projet\\resultats.txt";
            String filteredFilePath = "C:\\Users\\HP\\Documents\\projet\\resultats_filtree.txt";

try {
    // Command used to execute the Python script with the input and output files.
    String cmdPython = String.format("python \"%s\" \"%s\" \"%s\"", pythonScript, resultsFilePath, filteredFilePath);

    Process procPython = Runtime.getRuntime().exec(cmdPython);

    BufferedReader stdInputPython = new BufferedReader(new InputStreamReader(procPython.getInputStream()));
    BufferedReader stdErrorPython = new BufferedReader(new InputStreamReader(procPython.getErrorStream()));

    // Threads used to collect the Python script standard output and error streams.
    new Thread(() -> {
        String s;
        try {
            while ((s = stdInputPython.readLine()) != null) System.out.println(s);
        } catch (IOException ignored) {}
    }).start();

    new Thread(() -> {
        String s;
        try {
            while ((s = stdErrorPython.readLine()) != null) System.err.println(s);
        } catch (IOException ignored) {}
    }).start();

    // Wait for the Python script to finish before loading the filtered file.
    new Thread(() -> {
        try {
            procPython.waitFor();

            SwingUtilities.invokeLater(() -> {
                File filteredFilePathObj = new File(filteredFilePath);
                if (!filteredFilePathObj.exists() || filteredFilePathObj.length() == 0) {
                    JOptionPane.showMessageDialog(frame,
                            "<html><b>Error: Python filtering failed or the output file is empty.</b></html>",
                            "Filtering failed",
                            JOptionPane.ERROR_MESSAGE);
                    return;
                }

                // Load the filtered file in the 3D viewer.
                reconstructionDialog.dispose();
                surface3D.loadFile(filteredFilePath);
                coordLabel.setVisible(true);
                buttonsPanel.setVisible(true);
                cl.show(cards, "SURFACE3D");
            });
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
    }).start();

} catch (IOException ex) {
    JOptionPane.showMessageDialog(frame, "Error lancement script Python : " + ex.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
}

        });
    } catch (InterruptedException ex) {
        ex.printStackTrace();
    }
}).start();


    } catch (IOException ex) {
        scanDialog.dispose();
        JOptionPane.showMessageDialog(frame, "Error lancement scan : " + ex.getMessage(), "Error", JOptionPane.ERROR_MESSAGE);
    }
});
            dialog.setVisible(true);
        });


        loadFileButton.addActionListener(evt -> {
    JFileChooser fileChooser = new JFileChooser();
    fileChooser.setDialogTitle("Select a point file (.txt)");

    int result = fileChooser.showOpenDialog(frame);
    if (result == JFileChooser.APPROVE_OPTION) {
        File selectedFile = fileChooser.getSelectedFile();

        if (!selectedFile.exists()) {
            JOptionPane.showMessageDialog(frame, "File not found : " + selectedFile.getAbsolutePath(), "Error", JOptionPane.ERROR_MESSAGE);
            return;
        }

        surface3D.loadFile(selectedFile.getAbsolutePath());
        coordLabel.setVisible(true);
        buttonsPanel.setVisible(true);
        cl.show(cards, "SURFACE3D");
    }
});


        // Zoom buttons
        zoomIn.addActionListener(e -> {
            surface3D.zoom *= 1.2;
            surface3D.repaint();
        });
        zoomOut.addActionListener(e -> {
            surface3D.zoom /= 1.2;
            surface3D.repaint();
        });

        // Back button
        backButton.addActionListener(e -> {
            cl.show(cards, "HOME");
            coordLabel.setVisible(false);
            buttonsPanel.setVisible(false);
            surface3D.points.clear();
            surface3D.repaint();
        });

        // Rotation sliders.
        sliderX.addChangeListener(e -> {
            surface3D.angleX = Math.toRadians(sliderX.getValue());
            labelX.setText("Rotation X: " + sliderX.getValue() + "°");
            surface3D.repaint();
        });
        sliderY.addChangeListener(e -> {
            surface3D.angleY = Math.toRadians(sliderY.getValue());
            labelY.setText("Rotation Y: " + sliderY.getValue() + "°");
            surface3D.repaint();
        });
        sliderZ.addChangeListener(e -> {
            surface3D.angleZ = Math.toRadians(sliderZ.getValue());
            labelZ.setText("Rotation Z: " + sliderZ.getValue() + "°");
            surface3D.repaint();
        });

        frame.setVisible(true);
    }
}
