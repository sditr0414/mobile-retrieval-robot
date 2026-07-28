package com.mobile.retrieval.robot

import android.Manifest
import android.os.Build
import android.os.Bundle
import io.flutter.embedding.android.FlutterActivity

class MainActivity : FlutterActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Android 버전에 맞는 Bluetooth 런타임 권한을 앱 시작 시 요청한다.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            requestPermissions(
                arrayOf(
                    Manifest.permission.BLUETOOTH_CONNECT,
                    Manifest.permission.BLUETOOTH_SCAN,
                ),
                BLUETOOTH_PERMISSION_REQUEST,
            )
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            requestPermissions(
                arrayOf(Manifest.permission.ACCESS_FINE_LOCATION),
                BLUETOOTH_PERMISSION_REQUEST,
            )
        }
    }

    companion object {
        private const val BLUETOOTH_PERMISSION_REQUEST = 100
    }
}
