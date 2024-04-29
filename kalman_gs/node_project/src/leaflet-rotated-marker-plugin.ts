import Leaflet from 'leaflet';

(function () {
  // save these original methods before they are overwritten
  var proto_initIcon = (Leaflet.Marker.prototype as any)._initIcon;
  var proto_setPos = (Leaflet.Marker.prototype as any)._setPos;

  var oldIE = Leaflet.DomUtil.TRANSFORM === 'msTransform';

  Leaflet.Marker.addInitHook(function () {
    var iconOptions = this.options.icon && this.options.icon.options;
    var iconAnchor = iconOptions && this.options.icon.options.iconAnchor;
    if (iconAnchor) {
      iconAnchor = iconAnchor[0] + 'px ' + iconAnchor[1] + 'px';
    }
    this.options.rotationOrigin =
      this.options.rotationOrigin || iconAnchor || 'center bottom';
    this.options.rotationAngle = this.options.rotationAngle || 0;

    // Ensure marker keeps rotated during dragging
    this.on('drag', function (e) {
      e.target._applyRotation();
    });
  });

  Leaflet.Marker.include({
    _initIcon: function () {
      proto_initIcon.call(this);
    },

    _setPos: function (pos) {
      proto_setPos.call(this, pos);
      this._applyRotation();
    },

    _applyRotation: function () {
      if (this.options.rotationAngle) {
        this._icon.style[Leaflet.DomUtil.TRANSFORM + 'Origin'] =
          this.options.rotationOrigin;

        if (oldIE) {
          // for IE 9, use the 2D rotation
          this._icon.style[Leaflet.DomUtil.TRANSFORM] =
            'rotate(' + this.options.rotationAngle + 'deg)';
        } else {
          // for modern browsers, prefer the 3D accelerated version
          this._icon.style[Leaflet.DomUtil.TRANSFORM] +=
            ' rotateZ(' + this.options.rotationAngle + 'deg)';
        }
      }
    },

    setRotationAngle: function (angle) {
      this.options.rotationAngle = angle;
      this.update();
      return this;
    },

    setRotationOrigin: function (origin) {
      this.options.rotationOrigin = origin;
      this.update();
      return this;
    }
  });
})();
