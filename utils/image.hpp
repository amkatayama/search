#ifndef _IMAGE_HPP_
#define _IMAGE_HPP_

#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <boost/optional.hpp>

struct Color {
	Color(void) : r(1), g(1), b(1) { }
	Color(double _r, double _g, double _b) {
		setred(_r);
		setgreen(_g);
		setblue(_b);
	}
	void setred(double d) { r = clamp(d); }
	void setgreen(double d) { g = clamp(d); }
	void setblue(double d) { b = clamp(d); }
	double getred(void) const { return r; }
	double getgreen(void) const { return g; }
	double getblue(void) const { return b; }
	char getred255(void) const { return 255 * r; }
	char getgreen255(void) const { return 255 * g; }
	char getblue255(void) const { return 255 * b; }
private:
	static double clamp(double d) {
		if (d < 0.0) return 0.0;
		if (d > 1.0) return 1.0;
		return d;
	}
	double r, g, b;
};

struct Image {
	static const Color red, green, blue, white, black;

	Image(const char *title, unsigned int, unsigned int);

	~Image(void);

	void set(unsigned int x, unsigned int y, Color c) { data[y * w + x] = c; }
	void save(const char *, bool usletter = false) const;
	void output(FILE*, bool usletter = false) const;

	struct Component {
		virtual void write(FILE*) const = 0;
	};

	class Path : public Component {
		struct Point {
			Point(double _x, double _y) : x(_x), y(_y) { }
			double x, y;
		};

		typedef boost::optional<Point> CurLoc;

		struct Segment {
			virtual void write(FILE*) const = 0;
			virtual CurLoc move(CurLoc) const = 0;
		};

		struct MoveTo : Segment {
			MoveTo(double _x, double _y) : x(_x), y(_y) { }
			virtual void write(FILE*) const;
			virtual CurLoc move(CurLoc) const;
		private:
			double x, y;
		};

		struct LineTo : Segment {
			LineTo(double _x, double _y) : x(_x), y(_y) { }
			virtual void write(FILE*) const;
			virtual CurLoc move(CurLoc) const;
		private:
			double x, y;
		};

		struct SetLineWidth : Segment {
			SetLineWidth(double _w) : w(_w) { }
			virtual void write(FILE*) const;
			virtual CurLoc move(CurLoc) const;
		private:
			double w;
		};

		struct SetColor : Segment {
			SetColor(Color _c) : c(_c) { }
			virtual void write(FILE*) const;
			virtual CurLoc move(CurLoc) const;
		private:
			Color c;
		};

		struct Arc : Segment {
			Arc(double _x, double _y, double _r, double _t, double _dt) :
					x(_x), y(_y), r(_r), t(_t), dt(_dt) {
				if(dt > 360)
					dt = 360;
				else if(dt < -360)
					dt = - 360;
			}
			virtual void write(FILE*) const;
			virtual CurLoc move(CurLoc) const;
		protected:
			double x, y, r, t, dt;
		};

		struct NauticalArc : Arc {
			NauticalArc(double _x, double _y, double _r, double _t, double _dt) :
					Arc(_x, _y, _r, _t, -_dt) {
				t = Image::nautical2math(t);
			}
			friend class Image::Path;
		};

		void addseg(Segment *s) {
			endloc = s->move(endloc);
			segs.push_back(s);
		}

		CurLoc endloc;
		std::vector<Segment*> segs;

	public:
		~Path(void);

		void moveto(double x, double y) { addseg(new MoveTo(x, y)); }

		void lineto(double x, double y) { addseg(new LineTo(x, y)); }

		void setlinewidth(double x) { addseg(new SetLineWidth(x)); }

		void setcolor(Color c) { addseg(new SetColor(c)); }

		// Swings in the counter-clock-wise direction
		// from t degrees for dt degrees.m
		void arc(double x, double y, double r, double t, double dt) {
			addseg(new Arc(x, y, r, t, dt));
		}

		void nauticalarc(double x, double y, double r, double t, double dt) {
			addseg(new NauticalArc(x, y, r, t, dt));
		}

		void line(double x0, double y0, double x1, double y1);
		void curve(double xc, double yc, double r, double t, double dt);

		virtual void write(FILE*) const;
	};

	struct Text : public Component {
		enum { Left, Right, Centered };

		Text(const char *_text, unsigned int _x, unsigned int _y,
			double _sz = 12, Color _c = Image::black, int _pos = Centered,
			std::string _font = std::string("Times-Roman")) :
				x(_x), y(_y), sz(_sz), c(_c), pos(_pos), font(_font), text(_text) { }

		void setsize(double size) { sz = size; }
 
		void setcolor(Color color) { c = color; }

		void setfont(std::string f) { font = f; }

		void setpos(int p) { pos = p; }

		virtual void write(FILE*) const;

	private:
		unsigned int x, y;
		double sz;
		Color c;
		int pos;
		std::string font;
		std::string text;
	};

	struct Triangle : public Component {
		// Centered at (x,y) with height ht and width w
		// (an angle) rotated to point to at angle rot
		//
		// Negative line width means to fill in the triangle.
		Triangle(double _x, double _y, double _ht, double _w = 45,
				double _rot = 90, Color _c = Image::black,
				double _linewidth = 1) : x(_x), y(_y), w(_w), ht(_ht),
			rot(_rot), linewidth(_linewidth), c(_c) { }
		virtual void write(FILE*) const;
	private:
		double x, y, w, ht, rot;
		double linewidth;
		Color c;
	};

	struct Circle : public Component {
		Circle(double _x, double _y, double _r, Color _c = Image::black,
			bool _fill = false) : x(_x), y(_y), r(_r), c(_c), fill(_fill) { }
		virtual void write(FILE*) const;
	private:
		double x, y, r;
		Color c;
		bool fill;
	};

	void add(const Component *comp) {
		comps.push_back(comp);
	}

	static double nautical2math(double t) {
		if (t < 90)
			return 90 - t;
		else if (t < 180)
			return 360 - fmod(t, 90);
		else if (t < 270)
			return 270 - fmod(t, 90);
		return 180 - fmod(t, 90);
	}

private:

	/* 72/2 pt == ½ in */
	void outputhdr_usletter(FILE*, unsigned int marginpt = 72/2) const;
	void outputhdr(FILE*, unsigned int marginpt = 0) const;
	void outputdata(FILE*) const;
	void encodedata(std::string&) const;

	unsigned int w, h;
	std::string title;
	Color *data;
	std::vector<const Component *> comps;
};

#endif	// _IMAGE_HPP_