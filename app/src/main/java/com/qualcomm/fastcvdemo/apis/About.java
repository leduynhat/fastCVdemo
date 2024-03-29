/*==============================================================================
            Copyright (c) 2010-2011, 2014 Qualcomm Technologies Incorporated.
            All Rights Reserved.
            Qualcomm Technologies Confidential and Proprietary
==============================================================================*/

package com.qualcomm.fastcvdemo.apis;
 
import com.qualcomm.fastcvdemo.R;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.preference.PreferenceActivity;
import android.preference.PreferenceFragment;
import android.preference.PreferenceScreen;
import android.util.Log;
 
/**
 * About class. 
 * Some configurations for About menu is done in the xml file. 
 */
public class About extends PreferenceActivity 
{
   /** Logging tag */
   protected static final String         TAG               = "FastCVDemo";

   static
   {
      // Load JNI library
      Log.v( TAG, "About: load fastcvFeatDetect library");
      System.loadLibrary( "fastcvFeatDetect" );
   }

   private PreferenceScreen aboutScreen;

   /** Called when the activity is first created. */
   @Override
   protected void onCreate( Bundle savedInstanceState ) 
   {
      super.onCreate(savedInstanceState);
      addPreferencesFromResource( R.xml.about );
//      getFragmentManager().beginTransaction().replace(android.R.id.content, new AboutFragment()).commit();
      if (aboutScreen == null)
      {
         String fastcvVersion = getFastCVVersion();

         aboutScreen = getPreferenceScreen();

         PreferenceScreen versionNumberScreen = getPreferenceManager().createPreferenceScreen(this);
         versionNumberScreen.setTitle(getString(R.string.versionNumber_text));
         versionNumberScreen.setSummary(fastcvVersion);

         PreferenceScreen editorScreen = getPreferenceManager().createPreferenceScreen(this);
         editorScreen.setTitle("Last Edited by");
         editorScreen.setSummary("Le Duy Nhat");

         aboutScreen.addPreference(versionNumberScreen);
         aboutScreen.addPreference(editorScreen);
      }
      setPreferenceScreen(aboutScreen);
   }

   //Native Function Declarations   
   public native String getFastCVVersion();

   public static class AboutFragment extends PreferenceFragment
   {
      @SuppressLint("ResourceType")
      @Override
      public void onCreate(final Bundle savedInstanceState)
      {
         super.onCreate(savedInstanceState);
         addPreferencesFromResource(R.xml.about);
      }
   }
}
