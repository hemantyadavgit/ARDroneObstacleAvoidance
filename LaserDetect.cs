using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

using Emgu.CV;                  
using Emgu.CV.CvEnum;           
using Emgu.CV.Structure;        
using Emgu.Util;                

namespace CameraCapture
{
    class Vision
    {
        
        public Image<Gray, Byte> FindColor( Image<Bgr, Byte> src, int hue_low, int hue_high )
        {
            // Initialize HSV and Gray copies of image for color searching and thresholding
            Image<Hsv, Byte> img_HSV = src.Convert<Hsv, Byte>();
            Image<Gray, Byte> img_thresh = new Image<Gray, Byte>( CvInvoke.cvGetSize( src ) );

            // Initialize low and high color bounds for thresholding
            MCvScalar low =  new MCvScalar( hue_low,  100, 100 );
            MCvScalar high = new MCvScalar( hue_high, 255, 255 );

            // Threshold the HSV copy for input color range and output result in a Gray image
            CvInvoke.cvInRangeS( img_HSV, low, high, img_thresh );

            // Return image thresholded for input color
            return img_thresh;
        }
        //------------------------------------------------------------------------------------------

        //------------------------------------------------------------------------------------------
        // Find laser pointer dot in an image
        //------------------------------------------------------------------------------------------
        public Point FindLaser( Image<Gray, Byte> src )
        {
            double moment10, moment01;              // Moments used in locating laser point
            double area;                            // Central moment used in locating laser point
            Point laser = new Point();              // Laser pointer position
            MCvMoments moments = new MCvMoments();  // Moment object used for laser pointer location

            // Find initial moments
            CvInvoke.cvMoments( src, ref moments, 1 );

            // Find precise moments
            moment10 = CvInvoke.cvGetSpatialMoment( ref moments, 1, 0 );
            moment01 = CvInvoke.cvGetSpatialMoment( ref moments, 0, 1 );
            area = CvInvoke.cvGetCentralMoment( ref moments, 0, 0 );

            // Calculate laser point position
            laser.X = (int)( moment10 / area );
            laser.Y = (int)( moment01 / area );

            // Return laser pointer position
            return laser;
        }
        //------------------------------------------------------------------------------------------

        //------------------------------------------------------------------------------------------
        // Overlay an image on another image
        //------------------------------------------------------------------------------------------
        public void OverlayImage( Image<Bgr, Byte> src, Image<Bgr, Byte> over, Point loc,
                                  MCvScalar S, MCvScalar D )
        {
            for ( int x = 0; x < 640; x++ )
            {
                if ( x + loc.X > 640 ) continue;

                for ( int y = 0; y < 480; y++ )
                {
                    if ( y + loc.Y > 480 ) continue;

                    MCvScalar source = CvInvoke.cvGet2D( src, y + loc.Y, x + loc.X );
                    MCvScalar overlay = CvInvoke.cvGet2D( over, y, x );

                    MCvScalar merged;

                    merged.v0 = ( S.v0 * source.v0 + D.v0 * overlay.v0 );
                    merged.v1 = ( S.v1 * source.v1 + D.v1 * overlay.v1 );
                    merged.v2 = ( S.v2 * source.v2 + D.v2 * overlay.v2 );
                    merged.v3 = ( S.v3 * source.v3 + D.v3 * overlay.v3 );

                    CvInvoke.cvSet2D( src, y + loc.Y, x + loc.X, merged );
                }
            }

        }
        //------------------------------------------------------------------------------------------

        //------------------------------------------------------------------------------------------
        // Draw a crosshair on an image
        //------------------------------------------------------------------------------------------
        public void DrawCrosshair( int x, int y, MCvScalar color, Image<Bgr, Byte> src )
        {
            Point point_laser, point_left, point_right, point_top, point_bottom;

            // Set points for drawing crosshair
            point_laser = new Point( x, y );
            point_left = new Point( 0, y );
            point_top = new Point( x, 0 );
            point_right = new Point( CvInvoke.cvGetSize( src ).Width, y );
            point_bottom = new Point( x, CvInvoke.cvGetSize( src ).Height );

            // If image center tracking is desired
            /*
            point_laser = new Point( 320, 240 );
            point_left = new Point( 0, 240 );
            point_top = new Point( 320, 0 );
            point_right = new Point( CvInvoke.cvGetSize( src ).Width, 240 );
            point_bottom = new Point( 320, CvInvoke.cvGetSize( src ).Height );
            */

            // Draw a crosshair centered on the laser pointer on the webcam feed
            CvInvoke.cvCircle( src, point_laser, 5, color, 1,
                               Emgu.CV.CvEnum.LINE_TYPE.EIGHT_CONNECTED, 0 );
            CvInvoke.cvLine( src, point_left, point_right, color, 1,
                             Emgu.CV.CvEnum.LINE_TYPE.EIGHT_CONNECTED, 0 );
            CvInvoke.cvLine( src, point_top, point_bottom, color, 1,
                             Emgu.CV.CvEnum.LINE_TYPE.EIGHT_CONNECTED, 0 );
        }
        
        public void DrawMeasurements( int x, int y, double dist, double pfc,
                                      MCvScalar color, Image<Bgr, Byte> src )
        {
            // Measurement text content, position, and font
            string text_size, text_posn, text_pfc, text_dist;
            Point point_size, point_posn, point_pfc, point_dist;
            MCvFont font = new MCvFont( Emgu.CV.CvEnum.FONT.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5 );

            // Fill size string
            text_size = "Size (pix): ";
            text_size += Convert.ToString( CvInvoke.cvGetSize( src ).Width );
            text_size += ", ";
            text_size += Convert.ToString( CvInvoke.cvGetSize( src ).Height );

            // Start position, pfc, and distance strings
            text_posn = "Position (pix):  ";
            text_pfc  = "PFC (pix): ";
            text_dist = "Distance (cm): ";

            // If the laser point WAS found
            if ( ( x > 0 ) && ( y > 0 ) )
            {
                // Fill position string
                text_posn += Convert.ToString( x );
                text_posn += ", ";
                text_posn += Convert.ToString( y );

                // Fill pfc string
                text_pfc += Convert.ToString( pfc );

                // Fill distance string
                text_dist += String.Format( "{0:F1}", dist );
            }
            // If the laser pointer was NOT found
            else
            {
                // Fill measurement strings with NULL readings
                text_posn += "NULL, NULL";
                text_pfc += "NULL";
                text_dist += "NULL";
            }

            // Initialize text position
            point_size = new Point( 10, 400 );
            point_posn = new Point( 10, 420 );
            point_pfc  = new Point( 10, 440 );
            point_dist = new Point( 10, 460 );

            // Draw text on image
            CvInvoke.cvPutText( src, text_size, point_size, ref font, color );
            CvInvoke.cvPutText( src, text_posn, point_posn, ref font, color );
            CvInvoke.cvPutText( src, text_pfc,  point_pfc,  ref font, color );
            CvInvoke.cvPutText( src, text_dist, point_dist, ref font, color );
        }
        
        public Image<Bgr, Byte> GenerateMap( int steps, List<double> map, Image<Bgr, Byte> img_map )
        {
            Image<Hsv, Byte> img_heated = new Image<Hsv, Byte>( 640, 480 );
            Image<Bgr, Byte> img_ready;
            Hsv pixel;
            int index;
            double new_hue, max = 0, min = 1000000000, level = 0;
            Point origin = new Point( 0, 0 );
            MCvScalar S = new MCvScalar( 0.5, 0.5, 0.5, 0.5 );
            MCvScalar D = new MCvScalar( 0.5, 0.5, 0.5, 0.5 );

        
            for ( int n = 0; n < map.Count; n++ )
            {
                if ( map[ n ] > max )
                {
                    max = map[ n ];
                }
                if ( map[ n ] < min )
                {
                    min = map[ n ];
                }
            }

            level = ( max - min ) / 10;

            // Set each pixels satuation and brightness to the max
            for ( int n = 0; n < 480*640; n++ )
            {
                pixel = img_heated[ n/640, n%640 ];
                pixel.Satuation = 255;
                pixel.Value = 255;
                img_heated[ n/640, n%640 ] = pixel;
            }

            // Divide for each chunck of the image
            for ( int y = 0; y < 480; y += 480/steps )
            //for ( int y = 0; y < 480; y += 480 / 50 )
            {
                for ( int x = 0; x < 640; x += 640/steps )
                //for ( int x = 0; x < 640; x += 640 / 80 )
                {
                    // Set the hue to a color representing distance scaled to max distance
                    index = ( y / (480/steps) ) * ( steps ) + ( x / (640/steps) );
                    //index = ( y / ( 480 / 50 ) ) * ( steps ) + ( x / ( 640 / 80 ) );
                    new_hue = (double)120 * (map[ index ] - min) / (max - min);

                    // Pixel by pixel
                    for ( int yy = y; yy < y + 480/steps; yy++ )
                    {
                        for ( int xx = x; xx < x + 640/steps; xx++ )
                        {
                            pixel = img_heated[ yy, xx ];
                            pixel.Hue = new_hue;
                            img_heated[ yy, xx ] = pixel;
                        }
                    }
                }
            }

            // Convert the color map to the standard BGR
            img_ready = img_heated.Convert<Bgr, Byte>();
            // And overaly it on the start image with the input transparency
            //OverlayImage( img_map, img_ready, origin, S, D );

            return img_ready;
        }
        //------------------------------------------------------------------------------------------
    }
}